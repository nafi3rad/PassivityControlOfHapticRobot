/********************************************************************************
* This code is updated and modified by Nafise Faridi Rad. The implementation
* incorporates the theory of an adaptive energy reference time domain passivity
* controller for haptic interfaces into the jointTorqueCallback function, along
* with necessary changes throughout the code. This includes defining the
* optimization's constraint functions and introducing new structs 'my_func_data'
* and 'EnergyStruct'.


* The initialization of the robot, exception handling, and other aspects have
* the following rights:
*
* Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.
* OpenHaptics(TM) toolkit. The material embodied in this software and use of
* this software is subject to the terms and conditions of the clickthrough
* Development License Agreement.

* Description:
* This code initializes the PHANToM Omni haptic device which is interacting with a
* 2D virtual wall(in x, y direction) that has unstable behviaor(created by negative
* damping value). The adaptive energy reference is developed to create a stable
* reference energy which track closely the actual energy od the virtual environment.
* The parameter of the energy reference is optimized using NLopt library. The virtual
* damping 'alpha' changes the feedback force to operator to reach to a stable behavior
* within the haptic interface.

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <nlopt.h>
#include <tchar.h>
#include <math.h>
using namespace std;


#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
#include <time.h>
# include "conio.h"
# include <string.h>
#define FALSE 0
#define TRUE 1
#endif

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

//
HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);
HDSchedulerHandle hGravityWell = HD_INVALID_HANDLE;

void mainLoop(void);
bool initDemo(void);


//************* NLopt optimizer code******************************
/* The energy function is selected as Eref=0.5*Kx*Px^2 +0.5*Ky*Py^2 
The goal is to find parameters kx and ky with the aim of minimzing
error between E and Eref with the constraints: kx>0 and ky>0 to 
ensure stable behavior
*/
typedef struct {
	double E;
	double Px;
	double Py;
} my_func_data;

double myfunc(unsigned n, const double *x, double *grad, void *data)
{
	my_func_data *d = (my_func_data *)data;
	double r = d->E - x[0] * d->Px *  d->Px - x[1] * d->Py *  d->Py; //defining the objective function
	if (grad) {
		grad[0] = -2.0 * d->Px *  d->Px * r; // objective functions gradients
		grad[1] = -2.0 * d->Py *  d->Py * r;
	}

	return r * r;
}

//constrains on kx and ky
double myconstraint1(unsigned n, const double *x, double *grad, void *data)
{
	if (grad) {
		grad[0] = 0.0;
		grad[1] = -1.0; 
	}
	double r = -x[1];
	return r; // constraint is assumed to be r <= 0
}
double myconstraint2(unsigned n, const double *x, double *grad, void *data)
{
	if (grad) {
		grad[0] = -1.0; 
		grad[1] = 0.0;  
	}
	double r = -x[0];
	return r; // constraint is assumed to be r <= 0
}
//—————


void PrintHelp()
{
	static const char help[] = { \
		"CommandJointTorque Help\n\
		---\n\
		P: Prints device state\n\
		C: Continuously prints device state\n\
		H: Prints help menu\n\
		Q: Quits the program\n\
		---" };

	printf("\n%s\n", help);
}

/* Synchronization structure. */
typedef struct//defining a new type for collecting kinetic and kinematic parameters
{
	HDdouble forceValues[3];
	HDdouble jointTorqueValues[3];
	HDdouble gimbalTorqueValues[3];
	HDdouble positionValues[3];
	HDdouble velocityValues[3];

} DeviceStateStruct;

typedef struct//new type for the adaptive energy reference controller
{
	//______energy values
	HDdouble energy;
	HDdouble observedEnergy;
	//______virtual damping for reaching to the energy reference
	HDdouble alpha;
	HDdouble oldAlpha;
	//______Updating forces
	HDdouble oldFc;
	HDdouble doldFc;
	hduVector3Dd oldFpc;
	queue<hduVector3Dd> force_history;
	//_____Updating position and velocity
	HDdouble olddx;
	HDdouble oldx;
	hduVector3Dd oldVelocity;
	//______Updating paramters of the energy reference
	HDdouble oldkhat;
	HDdouble oldP;
	hduVector3Dd khp_vector;
	
	HDint counter;
	
} EnergyStruct;

/*for Saving Data in CSV files*/
ofstream csvEnergy;
ofstream csvTorque;
ofstream csvPosition;
ofstream csvVelocity;
ofstream csvSampling;
ofstream csvAlpha;
ofstream csvKhat;
ofstream csvControlForce;

/*****************************************************************************
Callback that retrieves state.
*****************************************************************************/
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
	DeviceStateStruct *pState = (DeviceStateStruct *)pUserData;//cast type

	hdGetDoublev(HD_CURRENT_FORCE, pState->forceValues); 
	hdGetDoublev(HD_CURRENT_JOINT_TORQUE, pState->jointTorqueValues);
	hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, pState->gimbalTorqueValues);
	hdGetDoublev(HD_CURRENT_POSITION, pState->positionValues);
	hdGetDoublev(HD_CURRENT_VELOCITY, pState->velocityValues);

	return HD_CALLBACK_DONE;
}

/*****************************************************************************
Printing device sates if user asks for it
*****************************************************************************/
void PrintDeviceState(HDboolean bContinuous)
{
	int i;
	DeviceStateStruct state;
	memset(&state, 0, sizeof(DeviceStateStruct));

	do
	{
		hdScheduleSynchronous(GetDeviceStateCallback, &state,
			HD_DEFAULT_SCHEDULER_PRIORITY);

		printf("\n");
		{
			//printf("Current Base Force Values (N):");
			for (i = 0; i < 3; i++)
			{
				printf("%f,", state.forceValues[i]);
				if (i == 2){
					cout << state.forceValues[i] << endl;
				}
				else
				{
					cout << state.forceValues[i] << ",";
				}
			}

			for (i = 0; i < 3; i++)
			{
				printf("%f,", state.positionValues[i]);
				if (i == 2){
					cout << state.positionValues[i] << endl;
				}
				else
				{
					cout << state.positionValues[i] << ",";
				}
			}
			//
			for (i = 0; i < 3; i++)
			{
				printf("%f,", state.velocityValues[i]);
				if (i == 2){
					cout << state.velocityValues[i] << endl;
				}
				else
				{
					cout << state.velocityValues[i] << ",";
				}
			}
		}

		if (bContinuous)
		{
			Sleep(1);
		}

	} while (!_kbhit() && bContinuous);

}

/*******************************************************************************
Main function.
Initializes the device, starts the schedule, creates a schedule callback
to handle forces, waits for the user to press a button, exits
the application.
*******************************************************************************/
int main(int argc, char* argv[])
{
	HDErrorInfo error;
	/* Initialize the device, must be done before attempting to call any hd
	functions. Passing in HD_DEFAULT_DEVICE causes the default device to be
	initialized. */
	HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		getch();
		return -1;
	}

	printf("Command Joint Torque Demo!\n");
	printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

	if (!initDemo())
	{
		printf("Demo Initialization failed\n");
		printf("Press any key to exit\n");
		getch();

	}
	//ofstream csvEnergy;
	csvEnergy.open("energy.csv");
	csvTorque.open("torque.csv");
	csvPosition.open("position.csv");
	csvVelocity.open("Velocity.csv");
	csvSampling.open("sampling.csv");
	csvAlpha.open("Alpha.csv");
	csvKhat.open("khat.csv");
	csvControlForce.open("ControlForce.csv");

	/*Setting initial conditions*/
	EnergyStruct e;
	e.counter = 0;
	e.energy = 0.0;
	e.observedEnergy = 0.0;
	e.oldAlpha = 0.0;
	e.oldVelocity[0] = 0.0;
	e.alpha = 0.0;
	e.oldkhat = 0.0;
	e.oldP = 0.1;
	e.oldFc = 0.0;
	e.oldFpc= { 0.0, 0.0, 0.0 };
	e.khp_vector = { 0.0, 0.0, 0.0 };
	e.oldx = 0.0;
	HDdouble maxStif;
	HDdouble maxDamp;


	/* Schedule the main callback that will render forces to the device. */
	hGravityWell = hdScheduleAsynchronous(
		jointTorqueCallback, &e,
		HD_MAX_SCHEDULER_PRIORITY);

	hdEnable(HD_FORCE_OUTPUT);
	hdSetSchedulerRate(1000);
	hdStartScheduler();
	
	/* Check for errors and abort if so. */
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to start scheduler");
		fprintf(stderr, "\nPress any key to quit.\n");
		return -1;
	}

	PrintHelp();

	/* Start the main application loop */
	mainLoop();
	hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &maxStif);
	cout << maxStif << endl;
	hdGetDoublev(HD_NOMINAL_MAX_DAMPING, &maxDamp);
	cout << maxDamp << endl;

	/* For cleanup, unschedule callback and stop the scheduler. */
	hdStopScheduler();
	hdUnschedule(hGravityWell);
	
	/* Disable the device. */
	hdDisableDevice(hHD);

	/*close files*/
	csvEnergy.close();
	csvTorque.close();
	csvPosition.close();
	csvVelocity.close();
	csvSampling.close();
	csvAlpha.close();
	csvKhat.close();
	csvControlForce.close();
	return 0;
}
/******************************************************************************
The main loop of execution.  Detects and interprets keypresses.  Monitors and
initiates error recovery if necessary.
******************************************************************************/
void mainLoop()
{
	int keypress;
	int nMotorIndex = 0;

	while (TRUE)
	{
		if (_kbhit())
		{
			keypress = getch();
			keypress = toupper(keypress);

			switch (keypress)//check yhe val
			{
			case 'P': PrintDeviceState(FALSE); break;
			case 'C': PrintDeviceState(TRUE); break;
			case 'H': PrintHelp(); break;
			case 'Q': return;
			default: PrintHelp(); break;//none of them
			}
		}

		/* Check if the scheduled callback has stopped running */
		if (!hdWaitForCompletion(hGravityWell, HD_WAIT_CHECK_STATUS))
		{
			fprintf(stderr, "\nThe main scheduler callback has exited\n");
			fprintf(stderr, "\nPress any key to quit.\n");
			getch();
			return;
		}
	}
}


/*******************************************************************************
Servo callback.
Called every servo loop tick.  Simulates a gravity well, which sucks the device
towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK jointTorqueCallback(void *data)
{
	EnergyStruct* ep = (EnergyStruct*)data;

	//_______properties of the virtual environment
	const HDdouble kStiffnessx = 0.1; /* N/mm */
	const HDdouble kStiffnessy = 0.1;
	const HDdouble kStiffnessxy = 0.02;/*N.s/mm*/

	const HDdouble bDampingx = -0.002;
	const HDdouble bDampingy = -0.002;//positive b is passive
	//___location of the virtual environment
	const HDdouble kForceInfluence = 50.0; /* mm */
	static const hduVector3Dd wellPos(0, 0, 0);


	const HDdouble sampleTime = 0.001;
	HDErrorInfo error;

	hduVector3Dd position;
	hduVector3Dd velocity;
	hduVector3Dd vp;//previous velocity
	HDdouble Eref;// Reference Energy
	HDdouble oldEnergy;
	HDint currentRate;
	hduVector3Dd force;
	hduVector3Dd controlForce;
	hduVector3Dd Fpc;
	hduVector3Dd positionTwell;
	hduVector3Dd jointAngles;
	HHD hHD = hdGetCurrentDevice();

	//optimization parameters
	HDdouble l;
	HDdouble p;
	HDdouble z;
	const HDdouble irr = 0.9;

	/* Begin haptics frame. */
	hdBeginFrame(hHD);

	/* Get the current position of the device. */
	hdGetDoublev(HD_CURRENT_POSITION, position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);
	hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
	hdGetIntegerv(HD_INSTANTANEOUS_UPDATE_RATE, &currentRate);

	memset(force, 0, sizeof(hduVector3Dd));
	memset(controlForce, 0, sizeof(hduVector3Dd));
	/*filter velocity*/
	velocity = irr * velocity + (1 - irr) * (ep->oldVelocity);

	/* >  positionTwell = wellPos-position  <
	Create a vector from the device position towards the virtual wall */
	hduVecSubtract(positionTwell, wellPos, position);

	/* If the device position is within virtual wall in x and y direction,
	apply the force in that direction*/
	if (positionTwell[0] < 0.0 || positionTwell[1] < 0.0) {
		// Computing the force in the virtual wall

		controlForce[0] = (positionTwell[0] < 0.0) ? (-bDampingx * velocity[0] + kStiffnessx * positionTwell[0] + kStiffnessxy * positionTwell[1]) : 0.0;
		controlForce[1] = (positionTwell[1] < 0.0) ? (-bDampingy * velocity[1] + kStiffnessy * positionTwell[1] + kStiffnessxy * positionTwell[0]) : 0.0;
		controlForce[2] = 0.0;
	}
	else {
		controlForce = { 0.0, 0.0, 0.0 }; 
	}
	/*Adding delay to the force in case we want to examine that*/

	//ep->force_history.push(controlforce);
	//if (ep->force_history.size() == 21){
	//	force = ep->force_history.front();
	//	ep->force_history.pop();
	//}

	force = controlForce;

	ep->oldVelocity[2] = 0.0;
	velocity[2] = 0.0;

	/**************calculating observed energy *************/
	if (ep->counter>0)
	{
		oldEnergy = ep->observedEnergy +
			ep->oldAlpha * dotProduct(ep->oldVelocity, velocity) * sampleTime;

		ep->observedEnergy += -dotProduct(force, velocity) * sampleTime +
			ep->oldAlpha * dotProduct(ep->oldVelocity, velocity) * sampleTime;

	}
	else
	{
		oldEnergy = 0.0;

		ep->observedEnergy = -dotProduct(force, velocity) * sampleTime;
	}

	/*Optimization solved using nlopt to find the parameters of the energy reference*/
	double kh[3];
	double minf;
	kh[0] = ep->khp_vector[0];
	kh[1] = ep->khp_vector[1];
	kh[2] = ep->khp_vector[2];

	if (oldEnergy < ep->observedEnergy){
	double lb[3] = { 0, -1.0e9, -1.0e9 };
	nlopt_opt opt;
	opt = nlopt_create(NLOPT_LD_SLSQP, 3);
	nlopt_set_lower_bounds(opt, lb);

	my_func_data fndata;
	fndata.Px = positionTwell[0];//positionx;
	fndata.Py = positionTwell[1]; //positiony;
	fndata.E = ep->observedEnergy;// Eactual;
	nlopt_set_min_objective(opt, myfunc, &fndata);
	nlopt_add_inequality_constraint(opt, myconstraint1, NULL, +1e-8);
	//nlopt_add_inequality_constraint(opt, myconstraint2, NULL, -1e-8);

	nlopt_set_xtol_rel(opt, 1e-4);


	if (nlopt_optimize(opt, kh, &minf) < 0){
	cout << "nlopt failed!";
	}

	ep->khp_vector[0] = kh[0];
	ep->khp_vector[1] = kh[1];
	ep->khp_vector[2] = kh[2];

	}
	//__________designing the energy reference using the estimated parameteres
	Eref = kh[0] * positionTwell[0] * positionTwell[0] + kh[1] * positionTwell[1] * positionTwell[1] + 2 * kh[2] * positionTwell[0] * positionTwell[1];


	//________calculating the damping paramter alpha, how much damping is needed to reach to the energy reference
	if (ep->counter > 0.0)
	{
		if (oldEnergy > ep->observedEnergy && ep->observedEnergy <= Eref && abs(velocity[0])>2)
		{
			ep->alpha = -(ep->observedEnergy - Eref) / (dotProduct(velocity, velocity) * sampleTime + 0.00001);
		}
		else
		{
			ep->alpha = 0.0;
		}
	}
	else
	{
		ep->alpha = 0.0;
	}
	
	//________determining the feedback force to the operator, and the passivity controller force'fpc' resulting from alph
	if (positionTwell[0]<0.0 || positionTwell[1]<0.0)
	{
		force -= ep->alpha*velocity;
		Fpc = ep->alpha*velocity;
	}
	else
	{
		Fpc = 0.0*vp;
		ep->alpha = 0.0;
	}

	
	ep->oldFpc = Fpc;

	/* updating variables*/
	ep->counter++;
	ep->oldVelocity = velocity;
	ep->oldAlpha = ep->alpha;

	/*printing variables into csv files*/
	csvEnergy << ep->observedEnergy << endl;
	csvPosition << position << endl;
	csvVelocity << velocity << endl;
	csvTorque << force << endl;
	csvSampling << Eref << endl;
	csvKhat << ep->khp_vector << endl;
	csvControlForce << controlForce << endl;

	/* Send the forces  to the device. */

	hdSetDoublev(HD_CURRENT_FORCE, force);

	hdEndFrame(hHD);


	/* Check for errors and abort the callback if a scheduler error
	is detected. */
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error,
			"Error detected while rendering gravity well\n");

		if (hduIsSchedulerError(&error))
		{
			return HD_CALLBACK_DONE;
		}
	}

	/* Signify that the callback should continue running, i.e. that
	it will be called again the next scheduler tick. */
	//csvEnergy.close();
	return HD_CALLBACK_CONTINUE;
}

bool initDemo(void)
{
	HDErrorInfo error;
	int calibrationStyle;
	printf("Calibration\n");

	hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
	if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL)
	{
		printf("Please prepare for starting the demo by \n");
		printf("placing the device at its reset position.\n\n");
		printf("Press any key to continue...\n");
		getch();
		return 1;
	}
	if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET)
	{
		printf("Please prepare for starting the demo by \n");
		printf("placing the device at its reset position.\n\n");
		printf("Press any key to continue...\n");

		getch();

		hdUpdateCalibration(calibrationStyle);
		if (hdCheckCalibration() == HD_CALIBRATION_OK)
		{
			printf("Calibration complete.\n\n");
			return 1;
		}
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error, "Reset encoders reset failed.");
			return 0;
		}
	}
}
/*****************************************************************************/

