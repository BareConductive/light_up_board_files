/*******************************************************************************

  Bare Conductive Light Up Board
  ------------------------------

  Bare Conductive code written by Stefan Dzisiewski-Smith.

  This work is licensed under a MIT license https://opensource.org/licenses/MIT

  Copyright (c) 2017, Bare Conductive

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

 *******************************************************************************/

#define DEV_MODE false // sets a few things to make development easier

#include <MPR121.h>
#include <Wire.h>

const uint8_t MPR121_addr = 0x5C;
const uint8_t MPR121_interrupt_pin = 2;

// these may well have to change for battery operation
const uint8_t MPR121_touch_threshold = 150;
const uint8_t MPR121_release_threshold = 100;
const uint8_t MPR121_short_threshold = 200; // used to detect shorted electrodes to set mode

// lower threshold for the test jig to work with a 10k pulldown
const uint8_t MPR121_test_touch_threshold = 40;
const uint8_t MPR121_test_release_threshold = 20;

const uint8_t num_leds = 6;
const uint8_t led_pins[num_leds] = { 9, 6, 10, 11, 3, 5 };
const uint8_t num_LED_PWM_levels = 11;

const uint32_t LED_PWM_map[num_LED_PWM_levels] = { 0, 6554, 13107, 19661, 26214, 32768, 39321, 45875, 52428, 58982, 65535 }; // gamma-correct in output function
const uint32_t tick_interval_millis = 133; // all values have to be multiplied by 8 to account for increase in PWM frequency - this corresponds to approx 60 fps
uint32_t current_pwm_levels[num_leds] = { 0, 0, 0, 0, 0, 0 };
uint32_t target_pwm_levels[num_leds] = { 0, 0, 0, 0, 0, 0 };
uint32_t iir_lpf_coefficient = 0; // valid 1 to 255, higher values give smoother fading between PWM values

uint32_t last_millis = 0;
uint32_t this_millis = 0;
uint32_t start_millis = 0;

const uint8_t num_electrodes = 6;

enum mode_t { init_failure, lamp_a, lamp_b, lamp_c, candle, die, spin, test } mode = lamp_b; // default mode is lamp_b
enum state_t { on, off } state = off; // for "on-off" type modes, default to off
const uint32_t startup_pause_millis = 7200; // all values have to be multiplied by 8 to account for increase in PWM frequency

// Lamp A
// electrodes shorted together set board mode
const uint8_t lamp_a_iir_coefficient = 225;
const uint8_t lamp_a_electrode = 0;
const uint8_t lamp_a_short_electrodes[2] = { 9, 10 };
const uint8_t lamp_a_open_electrodes[2]  = { lamp_a_electrode, 8 };


// Lamp B
// no shorted electrodes for lamp B
const uint8_t lamp_b_iir_coefficient = 225;
const uint8_t lamp_b_num_electrodes = 6;
bool lamp_b_rotary_position_detected;
#if DEV_MODE
  const uint8_t lamp_b_electrodes[lamp_b_num_electrodes] = { 10, 9, 8, 2, 1, 0 };
  const uint8_t lamp_b_open_electrodes[lamp_b_num_electrodes] = { 10, 9, 8, 2, 1, 0 };
#else
  const uint8_t lamp_b_electrodes[lamp_b_num_electrodes] = { 8, 9, 10, 0, 1, 2};
  const uint8_t lamp_b_open_electrodes[lamp_b_num_electrodes] = { 8, 9, 10, 0, 1, 2};
#endif

// Lamp C
const uint8_t lamp_c_num_electrodes = 2;
const uint8_t lamp_c_rising_iir_coefficient = 225;
const uint8_t lamp_c_falling_iir_coefficient = 235;
const uint8_t lamp_c_electrodes[lamp_c_num_electrodes] = { 1, 9 };
const uint8_t lamp_c_open_electrodes[2]  = { lamp_c_electrodes[0], lamp_c_electrodes[1] };
const uint8_t lamp_c_short_electrodes[2] = { 8, 2 };
const uint8_t lamp_c_min_diff = 5;
const uint8_t lamp_c_max_diff = 50;
bool lamp_c_ignore_next_touch = false;
const uint8_t lamp_c_baseline_reset_dwell_ticks = 5;
uint8_t lamp_c_baseline_reset_dwell_ctr = 0;

// Candle
const uint8_t candle_electrode = 0;
const uint8_t candle_short_electrodes[2] = { 1, 2 };
const uint8_t candle_open_electrodes[2]  = { candle_electrode, 8 };
const uint8_t candle_change_threshold = 15; // Q0.8 notation - probability of the candle animation changing
const uint8_t candle_iir_coefficient = 225;
const uint16_t candle_PWM_low_limit = 30000;
const uint16_t candle_PWM_high_limit = 65535;

// Spin
const uint8_t spin_iir_coefficient = 225;
const uint8_t spin_anim_ticks_per_frame = 3;
const uint8_t spin_anim_pwm_level_loading = 2;
const uint8_t spin_anim_pwm_level_continuous = 10; // much brighter LEDs in continuous mode
uint8_t spin_anim_tick_ctr = 0;
uint8_t spin_anim_frame_ctr = 2; // start top and centre
const uint8_t spin_electrode = 0;

// Die
const uint8_t die_iir_coefficient = 150;
const uint8_t die_electrode = 0;
const uint8_t die_short_electrodes[2] = { 8, 9 };
const uint8_t die_open_electrodes[1] = { die_electrode };
const uint8_t die_pwm_level = 2;
const uint8_t die_num_spins_before_reveal = 9;
const uint8_t die_anim_spins_before_slowdown = 3;
const uint8_t die_anim_ticks_per_frame_start = 2;
uint8_t die_anim_tick_ctr = 0;
uint8_t die_anim_ticks_per_frame = 1;
uint8_t die_anim_frame_ctr = 2; // start top and centre
uint8_t die_spin_ctr = 0;
uint8_t die_value = 0;

// Test
const uint8_t test_mode_pin = 21;
const uint8_t test_electrodes[num_electrodes] =  { 10, 9, 8, 2, 1, 0 };

// we're not using Serial in this sketch
// but we need to test that these pins are working at test time
const uint8_t rx_pin = 0;
const uint8_t tx_pin = 1;

// MPR121 error
const uint8_t mpr121_error_led = 4;
const uint32_t mpr121_error_blink_period_millis = 4000; // 1 Hz blink if MPR121 init failure - all values have to be multiplied by 8 to account for increase in PWM frequency
const uint8_t mpr121_error_on_pwm = LED_PWM_map[ spin_anim_pwm_level_loading ] >>8; // dim the error mode
const uint8_t mpr121_error_off_pwm = LED_PWM_map[ 0 ] >>8; // dim the error mode

void setup(){
  // set PWM frequencies to 2 or 4 kHz
  TCCR0B = TCCR0B & 0b11111000 | 0x02;
  TCCR1B = TCCR1B & 0b11111000 | 0x02;
  TCCR2B = TCCR2B & 0b11111000 | 0x02;

  for( uint8_t i=0; i<num_leds; i++ ){
    pinMode( led_pins[i], OUTPUT );
    digitalWrite( led_pins[i], LOW );
  }

  pinMode( test_mode_pin, INPUT_PULLUP );

  if( digitalRead( test_mode_pin ) == LOW ){
    mode = test; // test jig pulls this pin low
    digitalWrite( led_pins[ mpr121_error_led ], HIGH ); // switch the error LED on here - if MPR121 init fails it will never get turned off
  }

  if( !MPR121.begin( MPR121_addr, MPR121_touch_threshold, MPR121_release_threshold, MPR121_interrupt_pin )){
    mode = init_failure;
    // this mode is only likely if the address pin is incorrect - most failures result in a while(1) hang
    while( 1 ){
    }
  } else {
    MPR121.goFast();
    MPR121.updateAll();
  }

  if( mode != test ) {
    start_millis = millis();

    // wait for cap levels to stabilise
    // perform a short spin animation while this happens
    iir_lpf_coefficient = spin_iir_coefficient; // less smoothing for spin animation

    do {
      this_millis = millis();
      if(( this_millis - last_millis ) > tick_interval_millis ){
        last_millis = this_millis;
        if( spin_anim_tick_ctr ++ >= spin_anim_ticks_per_frame ){
          spin_anim_tick_ctr = 0;
          target_pwm_levels[ spin_anim_frame_ctr ] = LED_PWM_map[ 0 ];
          if( ++spin_anim_frame_ctr >= num_leds ){
            spin_anim_frame_ctr = 0;
          }
          target_pwm_levels[ spin_anim_frame_ctr ] = LED_PWM_map[ spin_anim_pwm_level_loading ];
        }
        update_all_IIR_LPF( current_pwm_levels, target_pwm_levels, iir_lpf_coefficient );
        write_all_leds( current_pwm_levels );

      }
    } while (( this_millis - start_millis ) <= startup_pause_millis );

    randomSeed(analogRead(0)); // good time to call this as noise is being produced elsewhere that will couple in

    set_all_uint32_t_array_same( target_pwm_levels, num_leds, LED_PWM_map[0] );
    set_all_uint32_t_array_same( current_pwm_levels, num_leds, LED_PWM_map[0] );

    write_all_leds( current_pwm_levels );

    MPR121.updateAll();

    #if DEV_MODE
      mode = die;
    #else
      if (( MPR121.getFilteredData( lamp_b_open_electrodes[0]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_b_open_electrodes[1]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_b_open_electrodes[2]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_b_open_electrodes[3]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_b_open_electrodes[4]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_b_open_electrodes[5]  ) >= MPR121_short_threshold )){
        mode = lamp_b;
      } else if (( MPR121.getFilteredData( lamp_a_short_electrodes[0] ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_a_short_electrodes[1] ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_a_open_electrodes[0]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_a_open_electrodes[1]  ) >= MPR121_short_threshold )){
        mode = lamp_a;
      } else if (( MPR121.getFilteredData( lamp_c_short_electrodes[0] ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_c_short_electrodes[1] ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_c_open_electrodes[0]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( lamp_c_open_electrodes[1]  ) >= MPR121_short_threshold )){
        mode = lamp_c;
      } else if (( MPR121.getFilteredData( candle_short_electrodes[0] ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( candle_short_electrodes[1] ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( candle_open_electrodes[0]  ) >= MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( candle_open_electrodes[1]  ) >= MPR121_short_threshold )){
        mode = candle;
      } else if (( MPR121.getFilteredData( die_short_electrodes[0]    ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( die_short_electrodes[1]    ) <  MPR121_short_threshold ) &&
                 ( MPR121.getFilteredData( die_open_electrodes[0]     ) >= MPR121_short_threshold )){
        mode = die;
      } else {
        mode = spin; // if we're in an unknown config, enter spin
      }
    #endif
  }


  switch(mode){
    case lamp_a:
      iir_lpf_coefficient = lamp_a_iir_coefficient;
      break;

    case lamp_b:
      iir_lpf_coefficient = lamp_b_iir_coefficient;
      break;

    case lamp_c:
      // we set this differently for Lamp C with a different rising and falling IIR coefficient
      break;

    case spin:
      state = on;  // we have to start with this mode "on" as an error indication
      iir_lpf_coefficient = spin_iir_coefficient;
      break;

    case candle:
      iir_lpf_coefficient = candle_iir_coefficient;
      break;

    case die:
      iir_lpf_coefficient = die_iir_coefficient;
      break;

    case test:
      // if we managed to init the MPR121, turn off the error LED
      digitalWrite( led_pins[ mpr121_error_led ], LOW );
      pinMode( rx_pin, INPUT );
      pinMode( tx_pin, OUTPUT );
      digitalWrite( tx_pin, LOW );
      MPR121.setTouchThreshold( MPR121_test_touch_threshold );
      MPR121.setReleaseThreshold( MPR121_test_release_threshold );
      break;

    default:
      break;
  }

}

void loop(){

  this_millis = millis();
  if(( this_millis - last_millis ) > tick_interval_millis ){

    last_millis = this_millis;

    MPR121.updateTouchData();

    // hold one electrode to enable level setting, move hand over other to set level
    switch( mode ){

      // single electrode toggle switch
      case lamp_a:
        if(MPR121.isNewTouch( lamp_a_electrode )){
          if(state == off){
            target_pwm_levels[ 0 ] = LED_PWM_map[ num_LED_PWM_levels - 1 ];
            state = on;
          } else {
            target_pwm_levels[ 0 ] = LED_PWM_map[ 0 ];
            state = off;
          }
        }

        update_single_IIR_LPF( current_pwm_levels, target_pwm_levels, iir_lpf_coefficient );
        write_all_leds_same( current_pwm_levels[ 0 ]);

        break;

      case lamp_b:

        // rotary dimmer to set overall LED brightness
        lamp_b_rotary_position_detected = false;

        // set PWM level by combination of electrodes touched - check electrode on its own first, then pair of electrodes together
        // priority encoded to prefer higher output values
        for ( uint8_t i = 0; i < ( num_LED_PWM_levels >> 1 ); i++ ){
          if( MPR121.getTouchData( lamp_b_electrodes[lamp_b_num_electrodes -1 -i] ) && !MPR121.getTouchData( lamp_b_electrodes[lamp_b_num_electrodes -2 -i] )){
            target_pwm_levels[ 0 ] = LED_PWM_map[num_LED_PWM_levels -1 -( i << 1 )];
            lamp_b_rotary_position_detected = true;
            break;
          } else if( MPR121.getTouchData( lamp_b_electrodes[lamp_b_num_electrodes -1 -i] ) && MPR121.getTouchData( lamp_b_electrodes[lamp_b_num_electrodes -2 -i] )){
            target_pwm_levels[ 0 ] = LED_PWM_map[num_LED_PWM_levels -2 -( i << 1 )];
            lamp_b_rotary_position_detected = true;
            break;
          }
        }

        // look to turn to lowest level only if we find no other position and electrode 0 is touched
        if( !lamp_b_rotary_position_detected && MPR121.getTouchData( lamp_b_electrodes[0] )){
          target_pwm_levels[ 0 ] =  LED_PWM_map[0];
        }

        update_single_IIR_LPF( current_pwm_levels, target_pwm_levels, iir_lpf_coefficient );

        write_all_leds_same( current_pwm_levels[ 0 ]);

        break;

      case lamp_c:

        // hold one electrode to enable level setting, move hand over other to set level
        // release first electrode to hold setting
        if( MPR121.isNewTouch( lamp_c_electrodes[0] )){
          if( lamp_c_ignore_next_touch ){
            // allows us to ignore one spurious touch using a boolean flag
            lamp_c_ignore_next_touch = false;
          } else {
            // if we have a new touch, let's set the baseline to the current filtered data value
            // this helps us have greater sensitivity in poorly grounded environments (e.g. power pack operation)

            // start by disabling auto-baseline copy at startup for all electrodes
            // as we don't want this for the electrode we're currently holding down
            MPR121.setCalibrationLock(CAL_LOCK_ENABLED);

            // wait a bit for the filtered data of the proximity sensor to settle
            // not ideal, but easier to do this here with a delay than in the main loop
            // also, we don't have to respond to anything else while this is happening
            for( uint8_t i=0; i < lamp_c_baseline_reset_dwell_ticks; i++ ){
              MPR121.updateAll();
              delay( tick_interval_millis );
            }

            // set the baseline value for the proximity sensor to it's current filtered data value
            // have to shift the filtered data down by two bits as filtered data is 10-bit and
            // baseline is 8-bit externally (although 10-bit on the MPR121)
            MPR121.setRegister( MPR121_E0BV + lamp_c_electrodes[1],  (uint8_t)( MPR121.getFilteredData( lamp_c_electrodes[1] ) >> 2 ) );

            // having changed the baseline value, we need a dummy update to allow the MPR121
            // to revalidate its filtered data registers
            MPR121.updateAll();

            // we also have to ignore the next "new touch" event as that is triggered
            // by us changing the baseline manually
            lamp_c_ignore_next_touch = true;
          }
        } else if( MPR121.getTouchData( lamp_c_electrodes[0] )){
          // if we're still holding down the "enable" sensor
          // we need baseline and filtered data for this mode
          MPR121.updateBaselineData();
          MPR121.updateFilteredData();

          // map PWM to difference between baseline and filtered data
          target_pwm_levels[ 0 ] =  map( constrain( MPR121.getBaselineData( lamp_c_electrodes[1] ) - MPR121.getFilteredData( lamp_c_electrodes[1] ), lamp_c_min_diff , lamp_c_max_diff ),
                                   lamp_c_min_diff, lamp_c_max_diff, LED_PWM_map[0] , LED_PWM_map[num_LED_PWM_levels-1] );
          if( current_pwm_levels[0] < target_pwm_levels[0] ){
            update_single_IIR_LPF( current_pwm_levels, target_pwm_levels, lamp_c_rising_iir_coefficient );
          } else {
            update_single_IIR_LPF( current_pwm_levels, target_pwm_levels, lamp_c_falling_iir_coefficient );
          }
          write_all_leds_same( current_pwm_levels[ 0 ]);
        }

        break;

      case spin:

        // spin animation if unit is in unkown state (controllable if you put it there deliberately)
        if(MPR121.isNewTouch( spin_electrode )){
          if(state == off){
            state = on;
          } else {
            set_all_uint32_t_array_same( target_pwm_levels, num_leds, LED_PWM_map[ 0 ]);
            state = off;
          }
        }

        if( state == on ){
          if( spin_anim_tick_ctr ++ >= spin_anim_ticks_per_frame ){
            spin_anim_tick_ctr = 0;
            target_pwm_levels[ spin_anim_frame_ctr ] = LED_PWM_map[ 0 ];
            if( ++spin_anim_frame_ctr >= num_leds ){
              spin_anim_frame_ctr = 0;
            }
            target_pwm_levels[ spin_anim_frame_ctr ] = LED_PWM_map[ spin_anim_pwm_level_continuous ];
          }
        }

        update_all_IIR_LPF( current_pwm_levels, target_pwm_levels, iir_lpf_coefficient );
        write_all_leds( current_pwm_levels );

        break;

      case candle:

        // controllable candle-like flicker animation - can be turned on or off
        if(MPR121.isNewTouch( candle_electrode )){
          if(state == off){
            state = on;
          } else {
            set_all_uint32_t_array_same( target_pwm_levels, num_leds, LED_PWM_map[ 0 ]);
            state = off;
          }
        }

        if(state == on) {
          for ( uint8_t i=0; i < num_leds; i++ ){
            if( random( 0, 256 ) <= candle_change_threshold ){
              target_pwm_levels[ i ] = random( candle_PWM_low_limit, candle_PWM_high_limit);
            }
          }
        }

        update_all_IIR_LPF( current_pwm_levels, target_pwm_levels, iir_lpf_coefficient );
        write_all_leds( current_pwm_levels );

        break;

      case die:

        // virtual 6-sided die - gives a "random" result each time
        if(MPR121.isNewTouch( die_electrode )){
          die_value = random( 0, 6);
          die_spin_ctr = 0;
          die_anim_ticks_per_frame = die_anim_ticks_per_frame_start;
          state = on;
        } else if( state == on ){
          // stop at the top of the shop after a set number of spins
          // except if die value is 3, which looks better if we step one frame forward before displaying
          // N.B. die value ranges from 0..5 - we display that + 1
          if(( die_spin_ctr < die_num_spins_before_reveal ) || ( die_anim_frame_ctr != ( die_value == 3 ? 4 : 3) )) {
            if( die_anim_tick_ctr ++ >= die_anim_ticks_per_frame ){
              die_anim_tick_ctr = 0;
              target_pwm_levels[ die_anim_frame_ctr ] = LED_PWM_map[ 0 ];
              if( ++die_anim_frame_ctr >= num_leds ){
                die_anim_frame_ctr = 0;
                if( die_spin_ctr >= die_anim_spins_before_slowdown ){
                  die_anim_ticks_per_frame++;
                }
                die_spin_ctr++;
              }
              target_pwm_levels[ die_anim_frame_ctr ] = LED_PWM_map[ die_pwm_level ];
            }
          } else {
            // zero the target array
            set_all_uint32_t_array_same( target_pwm_levels, num_leds, LED_PWM_map[0] );

            // this is an impementation of a code-to-pattern generator
            // built from a truth table
            if(( die_value == 1 ) || ( die_value == 5 )){
              target_pwm_levels[ 0 ] = LED_PWM_map[ die_pwm_level ];
            }
            if( die_value > 1 ){
              target_pwm_levels[ 1 ] = LED_PWM_map[ die_pwm_level ];
              target_pwm_levels[ 5 ] = LED_PWM_map[ die_pwm_level ];
            }
            if( die_value > 2 ){
              target_pwm_levels[ 2 ] = LED_PWM_map[ die_pwm_level ];
              target_pwm_levels[ 4 ] = LED_PWM_map[ die_pwm_level ];
            }
            if( die_value != 3 ){
              target_pwm_levels[ 3 ] = LED_PWM_map[ die_pwm_level ];
            }
            state = off;
          }
        }

        update_all_IIR_LPF( current_pwm_levels, target_pwm_levels, iir_lpf_coefficient );
        write_all_leds( current_pwm_levels );

        break;

      case test:
        // used during ATE at factory only

        MPR121.updateTouchData();
        for( uint8_t i=0; i<num_leds; i++ ){
          if( MPR121.getTouchData( test_electrodes[i] )){
            digitalWrite( led_pins[i], 1 );
          } else {
            digitalWrite( led_pins[i], 0 );
          }
        }

        // invert anything on RX and output on TX
        digitalWrite( tx_pin, !digitalRead( rx_pin ));
        break;

      default:
        // should never end up here
        break;
    }
  }
}

void set_all_uint32_t_array_same(uint32_t *array, uint32_t size, uint32_t value){
  for ( uint32_t i = 0; i < size; i++ ){
    array[ i ] = value;
  }
}

void write_all_leds_same( uint32_t pwm_value ){
  for( uint8_t i=0; i<num_leds; i++ ){
    // simple gamma correction (γ = 2.0)
    analogWrite( led_pins[ i ], (uint8_t)((( pwm_value >> 8 ) * ( pwm_value >> 8 )) >> 8 ));
  }
}
void write_all_leds( uint32_t *pwm_values ){
  for( uint8_t i=0; i<num_leds; i++ ){
    // simple gamma correction (γ = 2.0)
    analogWrite( led_pins[ i ], (uint8_t)((( pwm_values[ i ] >> 8 ) * ( pwm_values[ i ] >> 8 )) >> 8 ));
  }
}

uint32_t update_single_IIR_LPF( uint32_t *current, uint32_t *target, uint32_t coefficient){
  current[ 0 ] = (( current[ 0 ] * coefficient ) + ( target[ 0 ] * ( (uint32_t)256 - coefficient ))) >> 8;
}

uint32_t update_all_IIR_LPF( uint32_t *current, uint32_t *target, uint32_t coefficient){
  for( uint8_t i=0; i<num_leds; i++ ){
    current[ i ] = (( current[ i ] * coefficient ) + ( target[ i ] * ( (uint32_t)256 - coefficient ))) >> 8;
  }
}
