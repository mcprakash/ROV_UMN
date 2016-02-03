/*********************************************************************
Encoder source code

Functions to get encoder data using interrupts
**********************************************************************
Copyright(C)  <December 2015>  <Chandra P. Mangipudi>

This program is free software : you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#include "Arduino.h"
#include "base_fcns.h"
#include "yei.h"
#include "pololu_imu.h"
#include "encoder.h"

#define lin_encoder_scale_factor 0.009980
#define cen_rot_encoder_scale_factor 2*3.142/24000
#define rot_encoder_scale_factor 2*3.142/3928

volatile Encoder Enc[3] = { { 2, 3, 0, 1 }, { 4, 5, 0, 1 }, { 6, 7, 0, 1 } };

int drA,drB;

// Encoder 0 - Linear Encoder MakerBot MSK002
// Encoder 1 - Rotaty Encoder

// Encoder 0 PinA- Pin 2 of Arduino
// Encoder 0 PinB- Pin 3 of Arduino
// Encoder 1 PinA- Pin 4 of Arduino
// Encoder 1 PinB- Pin 5 of Arduino
// Encoder 2 PinA- Pin 6 of Arduino
// Encoder 2 PinB- Pin 7 of Arduino


// Waste some time during interrupt 
// Commented out used as time is very precious in ROV main code

void debounce(int del) {
	for (int k = 0; k<del; k++) {
		/* can't use delay in the ISR so need to waste some time
		perfoming operations, this uses roughly 0.1ms on uno  */
		k = k + 0.0 + 0.0 - 0.0 + 3.0 - 3.0;
	}
}

//***************************** Interrupts on Pins for Encoder 0 ************************//
void isrPinAEn0(){

	/* read pin B right away                                   */
	drB = digitalRead(Enc[0].pinB);

	/* possibly wait before reading pin A, then read it        */
	// debounce(Enc[0].del);
	drA = digitalRead(Enc[0].pinA);

	/* this updates the counter                                */
	if (drA == HIGH) {   /* low->high on A? */

		if (drB == LOW) {  /* check pin B */
			Enc[0].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[0].pos--;  /* going counterclockwise: decrement  */
		}

	}
	else {                       /* must be high to low on A */

		if (drB == HIGH) { /* check pin B */
			Enc[0].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[0].pos--;  /* going counterclockwise: decrement  */
		}

	} /* end counter update                                    */

} /* end ISR pin A Encoder 0                                 */



/* Interrupt Service Routine: change on pin B for Encoder 0  */
void isrPinBEn0(){

	/* read pin A right away                                   */
	drA = digitalRead(Enc[0].pinA);

	/* possibly wait before reading pin B, then read it        */
	// debounce(Enc[0].del);
	drB = digitalRead(Enc[0].pinB);

	/* this updates the counter                                */
	if (drB == HIGH) {   /* low->high on B? */

		if (drA == HIGH) { /* check pin A */
			Enc[0].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[0].pos--;  /* going counterclockwise: decrement  */
		}

	}
	else {                       /* must be high to low on B */

		if (drA == LOW) {  /* check pin A */
			Enc[0].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[0].pos--;  /* going counterclockwise: decrement  */
		}

	} /* end counter update */

} /* end ISR pin B Encoder 0  */

//***************************** Interrupts on Pins for Encoder 1 ************************//

void isrPinAEn1(){

	/* read pin B right away                                   */
	drB = digitalRead(Enc[1].pinB);

	/* possibly wait before reading pin A, then read it        */
	// debounce(Enc[1].del);
	drA = digitalRead(Enc[1].pinA);

	/* this updates the counter                                */
	if (drA == HIGH) {   /* low->high on A? */

		if (drB == LOW) {  /* check pin B */
			Enc[1].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[1].pos--;  /* going counterclockwise: decrement  */
		}

	}
	else { /* must be high to low on A                       */

		if (drB == HIGH) { /* check pin B */
			Enc[1].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[1].pos--;  /* going counterclockwise: decrement  */
		}

	} /* end counter update                                    */

} /* end ISR pin A Encoder 1                                 */


void isrPinBEn1(){

	/* read pin A right away                                   */
	drA = digitalRead(Enc[1].pinA);

	/* possibly wait before reading pin B, then read it        */
	// debounce(Enc[1].del);
	drB = digitalRead(Enc[1].pinB);

	/* this updates the counter                                */
	if (drB == HIGH) {   /* low->high on B? */

		if (drA == HIGH) { /* check pin A */
			Enc[1].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[1].pos--;  /* going counterclockwise: decrement  */
		}

	}
	else { /* must be high to low on B                       */

		if (drA == LOW) {  /* check pin A */
			Enc[1].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[1].pos--;  /* going counterclockwise: decrement  */
		}

	} /* end counter update                                    */

} /* end ISR pin B Encoder 1                                 */


//***************************** Interrupts on Pins for Encoder 2 ************************//
void isrPinAEn2(){

	/* read pin B right away                                   */
	drB = digitalRead(Enc[2].pinB);

	/* possibly wait before reading pin A, then read it        */
	// debounce(Enc[2].del);
	drA = digitalRead(Enc[2].pinA);

	/* this updates the counter                                */
	if (drA == HIGH) {   /* low->high on A? */

		if (drB == LOW) {  /* check pin B */
			Enc[2].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[2].pos--;  /* going counterclockwise: decrement  */
		}

	}
	else { /* must be high to low on A                       */

		if (drB == HIGH) { /* check pin B */
			Enc[2].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[2].pos--;  /* going counterclockwise: decrement  */
		}

	} /* end counter update                                    */

} /* end ISR pin A Encoder 2                                 */


/* Interrupt Service Routine: change on pin B for Encoder 2  */
void isrPinBEn2(){

	/* read pin A right away                                   */
	drA = digitalRead(Enc[2].pinA);

	/* possibly wait before reading pin B, then read it        */
	// debounce(Enc[2].del);
	drB = digitalRead(Enc[2].pinB);

	/* this updates the counter                                */
	if (drB == HIGH) {   /* low->high on B? */

		if (drA == HIGH) { /* check pin A */
			Enc[2].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[2].pos--;  /* going counterclockwise: decrement  */
		}

	}
	else { /* must be high to low on B                       */

		if (drA == LOW) {  /* check pin A */
			Enc[2].pos++;  /* going clockwise: increment         */
		}
		else {
			Enc[2].pos--;  /* going counterclockwise: decrement  */
		}

	} /* end counter update                                    */

} /* end ISR pin B Encoder 2                                 */

//***************************** Function to Initialize the encoders ************************//

void init_encoder()
{
	//Configure Each Input Pin for Encoder
	pinMode(Enc[0].pinA, INPUT);
	pinMode(Enc[0].pinB, INPUT);
	attachInterrupt(Enc[0].pinA, isrPinAEn0, CHANGE);
	attachInterrupt(Enc[0].pinB, isrPinBEn0, CHANGE);
	digitalWrite(Enc[0].pinA, HIGH);
	digitalWrite(Enc[0].pinB, HIGH);

	pinMode(Enc[1].pinA, INPUT);
	pinMode(Enc[1].pinB, INPUT);
	attachInterrupt(Enc[1].pinA, isrPinAEn1, CHANGE);
	attachInterrupt(Enc[1].pinB, isrPinBEn1, CHANGE);
	digitalWrite(Enc[1].pinA, HIGH);
	digitalWrite(Enc[1].pinB, HIGH);

	pinMode(Enc[2].pinA, INPUT);
	pinMode(Enc[2].pinB, INPUT);
	attachInterrupt(Enc[2].pinA, isrPinAEn2, CHANGE);
	attachInterrupt(Enc[2].pinB, isrPinBEn2, CHANGE);
	digitalWrite(Enc[2].pinA, HIGH);
	digitalWrite(Enc[2].pinB, HIGH);
}

//***************************** Function to get value from encoders ************************//

void get_encoder()
{
	nav.data.enc[0] = lin_encoder_scale_factor*((float)Enc[0].pos);
	nav.data.enc[1] = cen_rot_encoder_scale_factor*((float)Enc[1].pos);
	nav.data.enc[2] = rot_encoder_scale_factor*((float)Enc[2].pos);

}

void print_encoder()
{
	Serial.println(nav.data.enc[0], 4);
}
