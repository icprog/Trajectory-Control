// Console application program for Continuous DAC using KUSB3100
// and Encoder sensing using USB1
// VC++6.0 version

// 04/05/2010 Multiple Motor Control - PD

// 04/12/2010 Trajectory Control

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>
#include <math.h>

#include <olmem.h>
#include <olerrors.h>
#include <oldaapi.h>
#pragma comment(lib, "winmm.lib")

#include <USD_USB.h>

#define NUM_BUFFERS	4
#define STRLEN 80
#define DOF 2
#define TS	0.028f // 0.021f for single motor, 0.076f for three motors
#define n 200
#define PI	3.141593
#define number_H	6 // No of neurons in hidden layer (threshold not included)
#define number_I	4 // No of neurons in input layer (incl. threshold)

char str[STRLEN];
UINT encoding = 0, resolution = 0;
DBL max = 0, min = 0;
DBL gain = 1.0;
ULNG value;
ULNG samples;
HBUF hBuffer = NULL;
PDWORD pBuffer32 = NULL;
PWORD pBuffer = NULL;
//DASSINFO ssinfo;

//long value;

int InitKusb(void);
int InitDAC(void);
int InitADC(void);
int InitDI(void);
float Get_Theta(unsigned char);
float Derivative(float, float);
double KpFuzzyRule(double, double[]);
double TriangularMembershipFunc(double, double, double, double);
double TrapezoidalMembershipFunction(double, double, double, double, double);
double InferenceMembership(double, double, double);
void OverrallMembership(double[], double[], double[], double[], double[]);
double defuzzify(double[], double[]);
float sigmoid(float);


int DaOutput(DBL dacvalue, UINT channel);
float AdInput(UINT channel);
long DInput(UINT channel);

#define SHOW_ERROR(ecode) MessageBox(HWND_DESKTOP, olDaGetErrorString(ecode, str, STRLEN),\
	"Error", MB_ICONEXCLAMATION | MB_OK);

#define CHECKERROR(ecode) if ((board.status = (ecode)) !=OLNOERROR) {\
	SHOW_ERROR(board.status);\
	olDaReleaseDASS(board.hdass);\
	olDaTerminate(board.hdrvr);\
	return((UINT)NULL);}

typedef struct tag_board {
	HDEV hdrvr;
	HDASS hdass;
	ECODE status;
	HBUF hbuf;
	PWORD lpbuf;
	char name[MAX_BOARD_NAME_LENGTH];
	char entry[MAX_BOARD_NAME_LENGTH];
} BOARD;

typedef BOARD* LPBOARD;
typedef BOARD* LPBOARD2;

static BOARD board, board2;
static ULNG count = 0; // New for cont ADC

BOOL CALLBACK
GetDriver(LPSTR lpszName, LPSTR lpszEntry, LPARAM lParam)
{
	// Board
	LPBOARD lpboard = (LPBOARD)(LPVOID)lParam;

	//	lstrcpynW(lpboard->name, lpszName, MAX_BOARD_NAME_LENGTH-1);
	//	lstrcpynW(lpboard->entry, lpszEntry, MAX_BOARD_NAME_LENGTH-1);

	lpboard->status = olDaInitialize(lpszName, &lpboard->hdrvr);
	if (lpboard->hdrvr != NULL)
		/*		printf("\nERROR!!!\n");
		if (lpboard->hdrvr == 67) printf("ERROR! OL CANNOT OPEN DRIVER\n");
		else if (lpboard->hdrvr == 127) printf("ERROR! OL UNSUPPORTED SYSTEM\n");*/
		return FALSE;

	else
		return TRUE;

	// Board2

	LPBOARD2 lpboard2 = (LPBOARD2)(LPVOID)lParam;

	/*
	lstrcpyn(lpboard->name, lpszName, MAX_BOARD_NAME_LENGTH-1);
	lstrcpyn(lpboard->entry, lpszEntry, MAX_BOARD_NAME_LENGTH-1);

	lpboard->status = olDaInitialize(lpszName, &lpboard->hdrvr);
	if (lpboard->hdrvr != NULL)
	printf("\nERROR!!!\n");
	if (lpboard->hdrvr == 67) printf("ERROR! OL CANNOT OPEN DRIVER\n");
	else if (lpboard->hdrvr == 127) printf("ERROR! OL UNSUPPORTED SYSTEM\n");
	return FALSE;

	else
	return TRUE;*/
}

/*BOOL CALLBACK
InputBox(HWND hDlg,
UINT message,
WPARAM wParam,
LPARAM lParam)
*/

int main(int argc, char* argv[])
{

	UINT channel = 0;
	unsigned char ModuleAddress;

	int NumberOfUSB1s = 0;
	BOOL blnResult;

	//	unsigned long TimeStamp;
	//unsigned char ParallelInput;

	DWORD start, finish, duration;
	FILE *fp, *fn, *fw;
	int i, j, k, ii, jj;
	float tmp;
	const int m = 200;
	float e[3][m], u[3][m];
	float Kp[3] = { 3.0, 5.0, 0.0 }, Kd[3] = { 0.1, 0.1, 0.0 }; // Position control 
																//	float Kp[3] = {50.0, 3.0, 0.0}, Kd[3] ={1.00, 0.250, 0.0}; // Trajectory control 
	float theta[3][m], dtheta[3][m], de[3][m], thetad[3][m], dthetad[3][m], ddthetad[3][m];
	long lVal[3];
	double Kp_value[n], Kp_saturation = 50.0, KpVal[3][m];
	float c[3] = { 10.0f, 10.0f, 0.0f }, s[3][m], Phi_p[3] = { 10.0f, 6.0f, 0.0f }, Phi_d[3] = { 0.5f, 1.0f, 0.0f };
	float l = 1.0, h = 50.0, Enorm = 0.0f, Unorm = 0.0f;
	float H2IW[DOF][number_I][number_H];	// Hidden layer weights
	float O2HW[DOF][number_H + 1];	// Output layer weights 
	float H_Output[DOF][number_H + 1];
	float TInput[DOF][number_I];
	float NN_Output[DOF];

	if ((fw = fopen("TrainedWts.txt", "rt")) == NULL)
	{
		printf("Unable to open TrainedWeights file!\n"); exit(0);
	}

	for (k = 0; k < DOF; k++) {
		for (i = 0; i < number_I; i++)
		{
			for (j = 0; j < number_H; j++) fscanf(fw, "%f", &H2IW[k][i][j]);
		}
		for (i = 0; i < number_H + 1; i++)
		{
			fscanf(fw, "%f", &O2HW[k][i]);
		}
	}

	// ************ Position Control Reference Trajectories *********
/*
	for (i = 0; i < m; i++)
	{
		thetad[0][i] = 90.0f;
		thetad[1][i] = 90.0f;
		thetad[2][i] = 45.0f;

		for (j = 0; j < DOF; j++) {
			dthetad[j][i] = 0.0f;
			ddthetad[j][i] = 0.0f;
		}
	}
	*/

	//
	// ************ Trajectory Control Reference Trajectories: Cycloidal *********

	for (i=0; i < m; i++)
	{
	if (i < m/2.55)
	{
	thetad[0][i] = 45.0f *(TS*3*i - sin(TS*3*i))/(2 * PI);
	thetad[1][i] = 45.0f *(TS*3*i - sin(TS*3*i))/(2 * PI);
	thetad[2][i] = 45.0f *(TS*3*i - sin(TS*3*i))/(2 * PI);

	dthetad[0][i] = 40.0f *(3.0 - 3*cos(TS*3*i))/(2 * PI);
	dthetad[1][i] = 45.0f *(3.0 - 3*cos(TS*3*i))/(2 * PI);
	dthetad[2][i] = 45.0f *(3.0 - 3*cos(TS*3*i))/(2 * PI);

	ddthetad[0][i] = - 360.0f * sin(TS*3*i)/(2 * PI);
	ddthetad[1][i] = - 405.0f * sin(TS*3*i)/(2 * PI);
	ddthetad[2][i] = - 405.0f * sin(TS*3*i)/(2 * PI);
	} else
	{
	thetad[0][i] = 45.0f;
	thetad[1][i] = 45.0f;
	thetad[2][i] = 45.0f;

	dthetad[0][i] = dthetad[1][i] = dthetad[2][i] = 0.0f;
	ddthetad[0][i] = ddthetad[1][i] = ddthetad[2][i] = 0.0f;
	}

	}
	

	// ************ Trajectory Control Reference Trajectories: Sine *********

	// **********************************************************************


	NumberOfUSB1s = USB1Init();
	printf("Number of USB1s = %d\n", NumberOfUSB1s);

	blnResult = USB1ReturnModuleAddress(0, &ModuleAddress);
	if (blnResult == FALSE) printf("Cannot read Module Address!\n");

	InitKusb();

	for (i = 0; i < DOF; i++) {
		e[i][0] = 0.0;
		InitDAC();

		/*		printf("Initializing!\n");

		USB1GetIncPosition(ModuleAddress, i, &lVal[i]);
		if (i == 0 || i == 2) theta[i][0] = 360.0f*((float(lVal[i]%1600))/1600.0);
		else theta[i][0] = 360.0f*((float(lVal[i]%1250))/1250.0);
		while (theta[i][0] < 1.0f) DaOutput(2.0, channel+i);
		*/
		DaOutput(2.5, channel + i);
		USB1GetIncPosition(ModuleAddress, i, &lVal[i]);
		if (i == 0 || i == 2) theta[i][0] = 360.0f*((float(lVal[i] % 1600)) / 1600.0);
		else theta[i][0] = 360.0f*((float(lVal[i] % 1250)) / 1250.0);

	}
	printf("Initial angles: %10.3f\t%10.3f\n", 360.0f*((float(lVal[0] % 1600)) / 1600.0), 360.0f*((float(lVal[1] % 1250)) / 1250.0));
	for (i = 0; i < n; i++) Kp_value[i] = (i + 1) * (Kp_saturation) / n;

	printf("Press ANY key to start!\n"); getch();

	start = GetTickCount();


	for (i = 1; i <= m; i++)
	{
		l = l + 1.0;

		for (j = 0; j < DOF; j++)
		{
			USB1GetIncPosition(ModuleAddress, j, &lVal[j]);

			if (j == 0 || j == 2) theta[j][i] = 360.0f*((float(lVal[j] % 1600)) / 1600.0);
			else theta[j][i] = 360.0f*((float(lVal[j] % 1250)) / 1250.0);

			e[j][i] = thetad[j][i] - theta[j][i];
			dtheta[j][i] = Derivative(theta[j][i], theta[j][i - 1]);
			de[j][i] = dthetad[j][i] - dtheta[j][i];

			// ******************** Fuzzy Control ******************************

			Kp[j] = KpFuzzyRule(e[j][i] * 6, Kp_value) / 10.0;
			KpVal[j][i] = Kp[j];
			// *****************************************************************
			//
			/* ******************** Sliding Mode Control ***********************

			s[j][i] = c[j]*e[j][i] + de[j][i];
			if ((s[j][i]*e[j][i]) >= 0.0) Kp[j] = Phi_p[j];
			else Kp[j] = -Phi_p[j];
			if ((s[j][i]*de[j][i]) >= 0.0) Kd[j] = Phi_d[j];
			else Kd[j] = -Phi_d[j];

			*/
			u[j][i] = -(Kp[j] * e[j][i] + Kd[j] * de[j][i]) / 100;

			// ******** Calculate Feedforward Compensation from Neural Network ******

			// Calculate hidden layer neuron outputs

			TInput[j][0] = theta[j][i]; TInput[j][1] = dtheta[j][i]; TInput[j][2] = ddthetad[j][i];
			TInput[j][3] = 1.0;

			for (ii = 0; ii < number_H; ii++)
			{
				tmp = 0.0;
				for (k = 0; k <number_I; k++)
				{
					tmp = tmp + H2IW[j][k][ii] * TInput[ii][k];
				}
				H_Output[j][ii] = sigmoid(tmp);
			}
			H_Output[i][number_H] = 1.0f;

			// Calculate NN output and error

			for (j = 0; j < (number_H + 1); j++)
			{
				tmp = 0.0;
				tmp = tmp + O2HW[i][j] * H_Output[i][j];
			}
			NN_Output[i] = tmp;

			if (u[j][i] > 2.5) u[j][i] = 2.5; else if (u[j][i] < -2.5) u[j][i] = -2.5;

			InitDAC();
			DaOutput(u[j][i] + 2.5, channel + j);
		}
		Enorm = Enorm + e[0][i] * e[0][i] + e[1][i] * e[1][i]; Unorm = Unorm + u[0][i] * u[0][i] + u[1][i] * u[1][i];
	}

	finish = GetTickCount();
	duration = finish - start;

	Enorm = sqrt(Enorm); Unorm = sqrt(Unorm);
	printf("\nTS = %d msec\t Enorm = %10.3f\t Unorm = %10.3f\n", duration / m, Enorm, Unorm);
	printf("Initial angles: %10.3f\t%10.3f\n", 360.0f*((float(lVal[0] % 1600)) / 1600.0), 360.0f*((float(lVal[1] % 1250)) / 1250.0));

	for (j = 0; j < 3; j++) {

		InitDAC();
		DaOutput(2.5, channel + j);
	}

	if ((fp = fopen("P.txt", "wt")) == NULL)
	{
		printf("Cannot open the output file!\n");
	}
	if ((fn = fopen("NnetTrainingData.txt", "wt")) == NULL)
	{
		printf("Cannot open the output file!\n");
	}
	//	fprintf(fp, "\nTS = %d msec\t Enorm = %10.3f\t Unorm = %10.3f\n", duration/m, Enorm, Unorm);

	for (i = 1; i < m; i++)
	{

		fprintf(fp, "%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\t%10.3f\n", i*TS, thetad[0][i], theta[0][i], e[0][i], u[0][i], KpVal[1][i], thetad[1][i], theta[1][i], e[1][i], u[1][i], KpVal[1][i]);
		fprintf(fn, "%10.6f\t%10.6f\t%10.6f\t%10.6f\t%10.6f\t%10.6f\t%10.6f\t%10.6f\t%10.6f\t%10.6f\n", theta[0][i] / 57.3248, dtheta[0][i] / (40.0*57.3248), 0.0, 0.5*u[0][i], 1.0, theta[1][i] / 57.3248, dtheta[1][i] / (40.0*57.3248), 0.0, 0.5*u[1][i], 1.0);
	}
	rewind(fp);
	fclose(fp);

	InitDAC();
	DaOutput(2.5, channel + 1);

	olDaTerminate(board.hdrvr);

	std::cin.get();

	return((UINT)NULL);


}


int DaOutput(DBL dacvalue, UINT channel)
{
	float volts;
	long value;

	volts = (float)dacvalue;

	value = (long)((1L << resolution) / ((float)max - (float)min) * (volts - (float)min));
	value = min((1L << resolution) - 1, value);

	if (encoding != OL_ENC_BINARY) {
		long sign = 1L << (resolution - 1);
		value ^= sign;
		if (value & sign) value |= 0xffffffffL << resolution;
	}

	olDaPutSingleValue(board.hdass, value, channel, gain);
	olDaReleaseDASS(board.hdass);

	return 0;

}

float AdInput(UINT channel)
{
	float volts;
	long adcvalue;
	long value;

	CHECKERROR(olDaGetSingleValue(board.hdass, &adcvalue, channel, gain));

	if (encoding != OL_ENC_BINARY) {
		value ^= 1L << (resolution - 1);
		value &= (1L << resolution) - 1;
	}

	volts = ((float)max - (float)min) / (1L << resolution) * adcvalue;// + (float)min;
	olDaReleaseDASS(board.hdass);

	return volts;
}



float Get_Theta(unsigned char ModuleAddress)
{
	float theta;
	long lVal0;

	USB1GetIncPosition(ModuleAddress, 0, &lVal0);

	theta = 360.0f*((float(lVal0 % 3200)) / 3200.0);
	return theta;
}

float Derivative(float e1, float e0)
{
	float e_deri;
	e_deri = (e1 - e0) / TS;
	return e_deri;
}

