/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->k_p = Kpi;
   this->k_i = Kii;
   this->k_d = Kdi;
   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   // Since we already have a cte, we can use the old value to calculate the derivative of the error
   this->d_cte = (this->delta_time > 0) ? (cte - this->cte)/this->delta_time : 0.0;

   this->i_cte = cte*this->delta_time + this->i_cte;
   this->cte = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = - this->k_p*this->cte - this->k_d*this->d_cte - this->k_i*this->i_cte;

    if (control > this->output_lim_max) {
       control = this->output_lim_max;
    }
    else if (control < this->output_lim_min) {
       control = this->output_lim_min;
    }

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta_time = new_delta_time;
}