//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0



#include <stdio.h>
#include <math.h>
#include "dhdc.h"

#define REFRESH_INTERVAL  0.1   // sec

  // display instructions
  void printMenu(){
  printf ("press device 'BUTTON' or 'q' to quit\n");
  printf ("      'e' to make current position your wrist holder position\n");
  printf ("      'r' press if you rest your wrist in good position\n");
  //printf ("      'r' set your maximum angle point\n");
  //printf ("      't' switch gravity point to max angle point\n");
  //printf ("      'd' switch gravity point to base (wrist point)\n");
  printf ("      '1' switch angle measurement to horizontal\n");
  printf ("      '2' switch angle measurement to vertical\n");
  printf ("      'j' Turn on training mode - hand measurement tool\n");
  printf ("      'k' Turn off training mode \n");
  printf ("            |'n' reset measurement tool - moves hand to base position\n");
  printf ("            |'m' start force and angle measurement sequence\n");
  printf ("      'h' to print menu again\n");
  printf ("      '=' Measurement direction set clockwise \n");
  printf ("      '-' Measurement direction set counter-clockwise \n");
  printf ("            | if in training mode will work for training forces\n");
}

  

int
main (int  argc,
      char **argv)
{
  double px, py, pz;
  double fx, fy, fz;
  double t1,t0  = dhdGetTime ();
  int    done   = 0;
  double xd, yd, zd;
  double xt, yt, zt;
  double xn, yn, zn;
  double vx, vy, vz;
  double w_y, w_x, w_z;
  double radius;
  
  double rad_cor_table [3];
  double radius_correction_v_x, radius_correction_v_y, radius_correction_v_z;
  double force_correction_v_x, force_correction_v_y, force_correction_v_z;
  double force_correction_v_x_perp, force_correction_v_y_perp, force_correction_v_z_perp;
  double loose_diameter, radius_length, curr_radius_length, force_multiplier;
  double break_x, break_y, break_z;
  
  double force;
  double wrist_vector_x;
  double wrist_vector_y;
  double wrist_vector_z;
  double hand_vector_x;
  double hand_vector_y;
  double hand_vector_z;
  double angle_l;
  double angle_m;
  double angle_sum;
  double current_gravity_point_x, current_gravity_point_y, current_gravity_point_z;
  double max_angle_point_x, max_angle_point_y, max_angle_point_z;
  int    m_mode = 0; // flag responsible for current measure mode
  //m_mode possbile values: e - set start pos / r - start measurement /
  // t - go back to standby mode
  int    angle_mode = 0;
  // training mode
  int training_mode = 0;
  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);

  printf ("######################################################################################\n");
  printf ("#__        __   _                            _          _                 _   _      #\n");
  printf ("#\\ \\      / /__| | ___ ___  _ __ ___   ___  | |_ ___   | |__   __ _ _ __ | |_(_) ___ #\n");
  printf ("# \\ \\ /\\ / / _ \\ |/ __/ _ \\| '_ ` _ \\ / _ \\ | __/ _ \\  | '_ \\ / _` | '_ \\| __| |/ __|#\n");
  printf ("#  \\ V  V /  __/ | (_| (_) | | | | | |  __/ | || (_) | | | | | (_| | |_) | |_| | (__ #\n");
  printf ("#   \\_/\\_/ \\___|_|\\___\\___/|_| |_| |_|\\___|  \\__\\___/  |_| |_|\\__,_| .__/ \\__|_|\\___|#\n");
  printf ("#                                                                  |_| _             #\n");
  printf ("#         _ __ ___   ___  __ _ ___ _   _ _ __ ___ _ __ ___   ___ _ __ | |_           #\n");
  printf ("#        | '_ ` _ \\ / _ \\/ _` / __| | | | '__/ _ \\ '_ ` _ \\ / _ \\ '_ \\| __|          #\n");
  printf ("#        | | | | | |  __/ (_| \\__ \\ |_| | | |  __/ | | | | |  __/ | | | |_           #\n");
  printf ("#        |_| |_| |_|\\___|\\__,_|___/\\__,_|_|  \\___|_| |_| |_|\\___|_| |_|\\__|          #\n");
  printf ("#                                        _                                           #\n");
  printf ("#                          ___ _   _ ___| |_ ___ _ __ ___                            #\n");
  printf ("#                         / __| | | / __| __/ _ \\ '_ ` _ \\                           #\n");
  printf ("#                         \\__ \\ |_| \\__ \\ ||  __/ | | | | |                          #\n");
  printf ("#                         |___/\\__, |___/\\__\\___|_| |_| |_|                          #\n");
  printf ("#                              |___/                                                 #\n");
  printf ("######################################################################################\n");

                         
  // required to change asynchronous operation mode
  dhdEnableExpertMode ();

  // open the first available device
  if (dhdOpenID (1) < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  printMenu();
  // enable force
  dhdEnableForce (DHD_ON);
  
  //variables for measurement modes:
  double s_max_angle = 0;
  double s_max_force = 0;
  double measure_max_time = 3;
  double measure_cur_time=0;
  double test_time =0;
  
  //____speed limit controller values
  double p_gain = 80;
  double i_gain = 0;
  double d_gain = 0;
  double speed_break_point = 0.04;
  // set desired pos (gravity source point)
  double g_force_x, g_force_y, g_force_z;
  xd = 0.040; //starting gravity point
  yd = 0.0;   //
  zd = 0.0;   //
  max_angle_point_x=xd; max_angle_point_y=yd;
  current_gravity_point_x=xd; current_gravity_point_y=yd; current_gravity_point_z=zd;
  //set start forces
  xn = 0.0;
  yn = 0.0;
  zn = 0.0;
  //set start value of temp pos
  xt = 0.0;
  yt = 0.0;
  zt = 0.0;
  //set power of setting position
  force = 50;
  double training_force = 1.0;
  double hand_force_res = 0;
  //set wrist point
  w_x = 0.040;
  w_y = 0.0;
  w_z = 0.0;
  //set starting radius
  radius_length = 0;
  angle_l = 0;
  angle_m = 0;
  angle_sum = 0;
  
  wrist_vector_x = 0.0;
  wrist_vector_y = 0.0;
  wrist_vector_z = 0.0;
  
  hand_vector_x = 0.0;
  hand_vector_y = 0.0;
  hand_vector_z = 0.0;
  double minusone = -1; // -1 - used to change positive/negative number to opposite
  int curve_pow = 4; //change to controll the power of curve holding controller
  loose_diameter = 0;
  // loop while the button is not pushed
  while (!done) {
		
		
      //___________SUMM OF ALL MOVEMENT CONTROLLERS_______________________________________________
      if (dhdSetForceAndTorqueAndGripperForce (xn, yn, zn, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
      done = 1;
	  }
	  
		//________curve holding controller_________________________________
		//equations which provides correction vectors for aimed forces so
		//that the effector can be moved only on calculated curve
    if(angle_mode == 0){ //horizontal mode
		radius_length = sqrt(pow(max_angle_point_x-w_x,2)+pow(max_angle_point_y-w_y,2));
		curr_radius_length = sqrt(pow(px-w_x,2)+pow(py-w_y,2));
		
		//_______BLOCKING OUTER BORDER_______________
		if ((curr_radius_length - loose_diameter > radius_length) && (radius_length > 0) && (angle_mode == 0)){
			force_multiplier = pow((curr_radius_length - loose_diameter - radius_length)*100,curve_pow);
			radius_correction_v_x = (w_x - px)*force_multiplier;
			radius_correction_v_y = (w_y - py)*force_multiplier;
			}
		//_______BLOCKING INNER BORDER_______________
		else if((curr_radius_length < radius_length) && radius_length > 0 && angle_mode == 0){
			force_multiplier = pow((radius_length - curr_radius_length)*100,curve_pow);
			radius_correction_v_x = (px - w_x)*force_multiplier;
			radius_correction_v_y = (py - w_y)*force_multiplier;
			}
		//_______NEUTRAL FIELD______________________
		else{
			radius_correction_v_x = 0;
			radius_correction_v_y = 0;
			radius_correction_v_z = 0;
			}
		}
    else if(angle_mode == 1){ //vertical mode
		radius_length = sqrt(pow(max_angle_point_x-w_x,2)+pow(max_angle_point_z-w_z,2));
		curr_radius_length = sqrt(pow(px-w_x,2)+pow(pz-w_z,2));
		
		//_______BLOCKING OUTER BORDER_______________
		if ((curr_radius_length - loose_diameter > radius_length) && (radius_length > 0) && (angle_mode == 1)){
			force_multiplier = pow((curr_radius_length - loose_diameter - radius_length)*100,curve_pow);
			radius_correction_v_x = (w_x - px)*force_multiplier;
			radius_correction_v_z = (w_z - pz)*force_multiplier;
			}
		//_______BLOCKING INNER BORDER_______________
		else if((curr_radius_length < radius_length) && radius_length > 0 && angle_mode == 1){
			force_multiplier = pow((radius_length - curr_radius_length)*100,curve_pow);
			radius_correction_v_x = (px - w_x)*force_multiplier;
			radius_correction_v_z = (pz - w_z)*force_multiplier;
			}
		//_______NEUTRAL FIELD______________________
		else{
			radius_correction_v_x = 0;
			radius_correction_v_y = 0;
			radius_correction_v_z = 0;
			}
	}
    
    //________force  and resistance controller_________________________
    //this part will be controlling the forces and resistances that hand
    // can provide or resist during stress tests
    if((training_mode == 1) && radius_length > 0){
		
		//setting perpendicular vector
		//_______FORCE MEASUREMENT - standby__________
		if(py > 0){
			force_correction_v_x=(w_x - px);
			force_correction_v_y=(w_y - py);
			force_correction_v_x_perp =force_correction_v_y*angle_sum;
			force_correction_v_y_perp =(force_correction_v_x*minusone)*angle_sum;
			}
		else if(py<0){
			force_correction_v_x=(px - w_x);
			force_correction_v_y=(py - w_y);
			force_correction_v_x_perp =force_correction_v_y*angle_sum;
			force_correction_v_y_perp =(force_correction_v_x*minusone)*angle_sum;
			}
		hand_force_res = sqrt(pow(force_correction_v_x_perp,2)+pow(force_correction_v_y_perp,2));
		
		
	}	//_______RESISTANCE MEASUREMENT - active measurement_____
	else if(training_mode==2 && radius_length > 0){
		
		force_correction_v_x=(w_x - px);
		force_correction_v_y=(w_y - py);
		if(angle_sum<15){
			s_max_force+=0.001;
		}
		else{
			if(measure_cur_time<measure_max_time){
			measure_cur_time=measure_cur_time+0.0002;
			}
			else{
				printf("\r |Force that coused movement of the hand: %+0.03fN | Deflection angle aquired with this force: %+0.03f Deg|                                                                          \n", hand_force_res, angle_sum);
				measure_cur_time=0;
				training_mode=0;
			}
		}
		force_correction_v_x_perp =force_correction_v_y*training_force*s_max_force;
		force_correction_v_y_perp =(force_correction_v_x*minusone)*training_force*s_max_force;
		hand_force_res = sqrt(pow(force_correction_v_x_perp,2)+pow(force_correction_v_y_perp,2));
		
		
		
	}	//_______TRAINING TURNED OFF________
	else if(training_mode==0 && radius_length > 0){
		force_correction_v_x_perp = 0;
		force_correction_v_y_perp = 0;
		hand_force_res = 0;
		s_max_force=0;
	}
	
	
	//_______________Speed limit controller___________________
	dhdGetLinearVelocity (&vx, &vy, &vz);
	//____x controller_____
	if(vx>speed_break_point){
		break_x=(vx-speed_break_point)*(-p_gain);
		}
	if(vx<-speed_break_point){
		break_x=(vx+speed_break_point)*(-p_gain);
		}
	//____Y controller_____
	if(vy>speed_break_point){
		break_y=(vy-speed_break_point)*(-p_gain);
		}
	if(vy<-speed_break_point){
		break_y=(vy+speed_break_point)*(-p_gain);
		}
	//____Z controller_______
	if(vz>speed_break_point){
		break_z=(vz-speed_break_point)*(-p_gain);
		}
	if(vz<-speed_break_point){
		break_z=(vz+speed_break_point)*(-p_gain);
		}
    //count difference from desired position for gravity control and 
    // adds up forces calculated from different controllers
	  if(angle_mode == 0){
		g_force_x=0; g_force_y=0; g_force_z=70;
	  }
	  else if(angle_mode ==1){
		g_force_x=0; g_force_y=70; g_force_z=0;
	  }
      xn = 0*(current_gravity_point_x-xt)*force*1.4 + radius_correction_v_x*force + force_correction_v_x_perp + break_x; 
      yn = 0*(current_gravity_point_y-yt)*g_force_y + radius_correction_v_y*force + force_correction_v_y_perp + break_y;
      zn = 0*(current_gravity_point_z-zt)*g_force_z + radius_correction_v_z*force + break_z;
      
      if (dhdGetPosition (&px, &py, &pz) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;}
        
      xt = px;
      yt = py;
      zt = pz;
      
    //angle_measure
    if (angle_mode == 0){
      wrist_vector_x = 0.0-w_x;
      wrist_vector_y = 0.0-w_y;
      hand_vector_x = px-w_x;
      hand_vector_y = py-w_y;
      angle_l= (wrist_vector_x*hand_vector_x+wrist_vector_y*hand_vector_y);
      angle_m = (sqrt(pow(wrist_vector_x,2)+pow(wrist_vector_y,2))*sqrt(pow(hand_vector_x,2)+pow(hand_vector_y,2)));
      angle_sum = acos(angle_l/angle_m)*180.0/3.14159;
	}
	else{
	  wrist_vector_x = 0.0-w_x;
      wrist_vector_z = 0.0-w_z;
      hand_vector_x = px-w_x;
      hand_vector_z = pz-w_z;
      angle_l= (wrist_vector_x*hand_vector_x+wrist_vector_z*hand_vector_z);
      angle_m = (sqrt(pow(wrist_vector_x,2)+pow(wrist_vector_z,2))*sqrt(pow(hand_vector_x,2)+pow(hand_vector_z,2)));
      angle_sum = acos(angle_l/angle_m)*180.0/3.14159;
	}
      
    //printf ("Value of yn = %f\n",yn);
    // display refresh rate and position at 10Hz
    t1 = dhdGetTime ();
    if ((t1-t0) > REFRESH_INTERVAL) {
      
      const char *mode = "SAVN";

      // retrieve position
      if (dhdGetPosition (&px, &py, &pz) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
        
      }
      
      // retrieve force
      if (dhdGetForce (&fx, &fy, &fz) < DHD_NO_ERROR) {
        printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

      // display status
      printf ("Mode: %d p(%+0.03f %+0.03f %+0.03f)m | f(%+0.01f %+0.01f %+0.01f)N | Wrist Angle %+0.03f|force=%+0.03f|Resist/power=%+0.03fN | velocity(x:%+0.03f,y:%+0.03f,z:%+0.03f)\r",m_mode, px, py, pz, fx, fy, fz, angle_sum,force,hand_force_res,vx,vy,vz);

      // test for exit condition
      if (dhdGetButtonMask()) done = 1;
      if (dhdKbHit()) {
        switch (dhdKbGet()) {
        case 'q': done = 1; break;
        case 's': dhdSetComMode (DHD_COM_MODE_SYNC);  break;
        case 'a': dhdSetComMode (DHD_COM_MODE_ASYNC); break;
        case 'e': m_mode = 1; current_gravity_point_x = px; w_x = px; current_gravity_point_y = py; w_y = py; current_gravity_point_z = pz; w_z = pz; printf("\r Current position set as new start position.                                                                 \n"); break;
        case 'r': m_mode = 2; max_angle_point_x=px; max_angle_point_y=py; max_angle_point_z=pz; printf("\r Wrist rest position set to current placement                                                                                                                \n"); break;
        case 't': m_mode = 0; current_gravity_point_x=max_angle_point_x; current_gravity_point_y=max_angle_point_y; current_gravity_point_z=max_angle_point_z; printf("\r Gravity point changed to max angle point.                                                                       \n"); break;
        case 'd': m_mode = 3; current_gravity_point_x=xd; current_gravity_point_y=yd; current_gravity_point_z=zd; printf("\r Gravity point changed to base wrist point.                                                                      \n");break;
        case '1': angle_mode = 0; printf("\r Angle mode =0 Horizontal angle measurement                                                                               \n"); break;
        case '2': angle_mode = 1; printf("\r Angle mode =1 Vertical angle measurement                                                                                 \n"); break;
        case 'j': training_mode = 1; printf("\r Training mode turned on                                                                                                                              \n"); break;
        case 'k': training_mode = 0; printf("\r Training mode turned off                                                                                                                             \n"); break;
        case 'n': training_mode = 1; printf("\r Training mode =1 - force measurement                                                                                                                       \n"); break;
        case 'm': training_mode = 2; printf("\r Starting measurement sequence, wait untill force is realised                                                                                               \n"); break;
        case 'h': printf("\r                                                                                                                            \n"); printMenu(); break;
        case '=': if(training_mode==0){force=force+10; printf("\r Force increased by value of 10                                                                                                                               \n");}else{training_force=1;  printf("\r Force measurement clockwise                                                                                                                             \n");} break;
        case '-': if(training_mode==0){force=force-10; printf("\r Force decreased by value of 10                                                                                                                               \n");}else{training_force=-1; printf("\r Force measurement counter-clockwise                                                                                                                      \n");} break;
        }
      }

      // update timestamp
      t0 = t1;
    }
  }

  // close the connection
  dhdClose ();

  // happily exit
  printf ("\ndone.\n");
  return 0;
}
