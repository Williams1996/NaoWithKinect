#include <iostream>
#include <string.h>
#include <Windows.h>
#include <NuiApi.h>
#include <qi/anyobject.hpp>
#include <qi/applicationsession.hpp>
#include <boost/make_shared.hpp>
#include <vector>
#include "opencv2/opencv.hpp"
#include <fstream>
#include <math.h>

using namespace qi;
using namespace std;
using namespace cv;

void drawSkeleton(Mat &image, CvPoint pointSet[], int whichone);
double getAngle(Vector4 &vector1, Vector4 &vector2);
Vector4 getDiff(Vector4 &vector1, Vector4 &vector2);
Vector4 setToZero(Vector4 &vector, char m);
int getTime(){
	return clock() / CLOCKS_PER_SEC;
}


int main(int argc,char* argv[])
{
	//nao�����˵�׼��
	ApplicationSession app(argc, argv);
	app.start();
	SessionPtr session = app.session();

	AnyObject motion = session->service("ALMotion");
	AnyObject posture = session->service("ALRobotPosture");
	motion.call<void>("wakeUp");
	posture.call<bool>("goToPosture", "StandInit", 0.8f);
	//Kinect׼��
	ofstream thrdataNao, thrdataKinect;

	Mat skeletonImage;
	skeletonImage.create(240, 320, CV_8UC3);
	CvPoint skeletonPoint[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT] = { cvPoint(0, 0) };
	bool tracked[NUI_SKELETON_COUNT] = { FALSE };
	Vector4 coordnateOfNao[20];
	double angles[21] = {0.0};
	int t = 0;
	int lastT = 0;
	//1����ʼ��NUI��ע��������USES_SKELETON  
	HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
	if (FAILED(hr))
	{
		cout << "NuiInitialize failed" << endl;
		return hr;
	}
	else
	{
		cout << "NuiInitialize Succeed" << endl;
	}

	//2����������ź��¼����   
	HANDLE skeletonEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	//3���򿪹��������¼�  
	hr = NuiSkeletonTrackingEnable(skeletonEvent, 0);
	if (FAILED(hr))
	{
		cout << "Could not open color image stream video" << endl;
		NuiShutdown();
		return hr;
	}
	else
	{
		cout << "Open color image stream video succeed." << endl;

	}
	namedWindow("skeletonImage", CV_WINDOW_AUTOSIZE);
	thrdataKinect.open("3dlocationKinect.txt");
	thrdataNao.open("3dlocationNao.txt");
	//4����ʼ��ȡ������������   
	while (1)
	{
		//int now = getTime();
		//if (now - lastT > 0)
		//{
		for (int k = 0; k <= 21; k++)
		{
			angles[k] = 0.0;
		}
			NUI_SKELETON_FRAME skeletonFrame = { 0 };  //����֡�Ķ���   
			bool bFoundSkeleton = false;

			//4.1�����޵ȴ��µ����ݣ��ȵ��󷵻�  
			if (WaitForSingleObject(skeletonEvent, INFINITE) == 0)
			{
				//4.2���ӸղŴ���������������еõ���֡���ݣ���ȡ�������ݵ�ַ����skeletonFrame  
				hr = NuiSkeletonGetNextFrame(0, &skeletonFrame);
				if (SUCCEEDED(hr))
				{
					//NUI_SKELETON_COUNT�Ǽ�⵽�Ĺ��������������ٵ���������  
					for (int i = 0; i < NUI_SKELETON_COUNT; i++)
					{
						NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
						//4.3��Kinect����������ˣ���ֻ�ܸ��������˵Ĺ������ټ��ÿ�����ˡ����п����ǿգ������ˣ�  
						//�Ƿ���ٵ���   
						if (trackingState == NUI_SKELETON_TRACKED)
						{
							bFoundSkeleton = true;
						}
						//cout << "The number of persons is :" << i << endl;
					}
				}

				if (!bFoundSkeleton)
				{
					continue;
				}


				//4.4��ƽ������֡����������  
				NuiTransformSmooth(&skeletonFrame, NULL);
				skeletonImage.setTo(0);

				for (int i = 0; i < NUI_SKELETON_COUNT; i++)
				{
					// Show skeleton only if it is tracked, and the center-shoulder joint is at least inferred.   
					//�϶��Ƿ���һ����ȷ���������������������ٵ����Ҽ粿���ģ�����λ�ã�������ٵ���   
					if (skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED &&
						skeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] != NUI_SKELETON_POSITION_NOT_TRACKED)
					{
						cout << "This is a person." << endl;
						float fx, fy;
						//�õ����и��ٵ��Ĺؽڵ�����꣬��ת��Ϊ���ǵ���ȿռ�����꣬��Ϊ�����������ͼ����  
						//����Щ�ؽڵ��ǳ�����  
						//NUI_SKELETON_POSITION_COUNTΪ���ٵ���һ�������Ĺؽڵ����Ŀ��Ϊ20  
						for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
						{
							NuiTransformSkeletonToDepthImage(skeletonFrame.SkeletonData[i].SkeletonPositions[j], &fx, &fy);
							skeletonPoint[i][j].x = (int)fx;
							skeletonPoint[i][j].y = (int)fy;
							cout << "j=" << j << endl;
							thrdataKinect << "i=" << i << ",j=" << j << ":" << skeletonFrame.SkeletonData[i].SkeletonPositions[j].w <<
								"," << skeletonFrame.SkeletonData[i].SkeletonPositions[j].x << "," << skeletonFrame.SkeletonData[i].SkeletonPositions[j].y
								<< "," << skeletonFrame.SkeletonData[i].SkeletonPositions[j].z << endl;
							if (j == 0)
							{
								coordnateOfNao[j].x = 0.0;
								coordnateOfNao[j].y = 0.0;
								coordnateOfNao[j].z = 0.0;
								coordnateOfNao[j].w = 0.0;
							}
							else
							{
								coordnateOfNao[j].x = skeletonFrame.SkeletonData[i].SkeletonPositions[j].z - skeletonFrame.SkeletonData[i].SkeletonPositions[0].z;
								coordnateOfNao[j].y = skeletonFrame.SkeletonData[i].SkeletonPositions[j].x - skeletonFrame.SkeletonData[i].SkeletonPositions[0].x;
								coordnateOfNao[j].z = skeletonFrame.SkeletonData[i].SkeletonPositions[j].y - skeletonFrame.SkeletonData[i].SkeletonPositions[0].y;
								coordnateOfNao[j].w = skeletonFrame.SkeletonData[i].SkeletonPositions[j].w - skeletonFrame.SkeletonData[i].SkeletonPositions[0].w;
							}
							//thrdataNao << "i=" << i << ",j=" << j << ":" << coordnateOfNao[j].x << "," << coordnateOfNao[j].y << "," << coordnateOfNao[j].z
								//<< "," << coordnateOfNao[j].w << endl;
						}

						for (int j = 0; j<NUI_SKELETON_POSITION_COUNT; j++)
						{
							if (skeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[j] != NUI_SKELETON_POSITION_NOT_TRACKED)//���ٵ�һ��������״̬��1û�б����ٵ���2���ٵ���3���ݸ��ٵ��Ĺ��Ƶ�     
							{
								circle(skeletonImage, skeletonPoint[i][j], 3, cvScalar(0, 255, 255), 1, 8, 0);
								tracked[i] = TRUE;
							}
						}

						drawSkeleton(skeletonImage, skeletonPoint[i], i);

						double angleTemp[21] = { 0.0 };
						for (int k = 0; k <= 21; k++)
						{
							angleTemp[k] = 0.0;
						}
						
						
						angleTemp[0] = getAngle(getDiff(coordnateOfNao[5], coordnateOfNao[4]), setToZero(getDiff(coordnateOfNao[5], coordnateOfNao[4]), 'z'));
						if (getDiff(coordnateOfNao[5], coordnateOfNao[4]).z > 0.0)
						{
							angles[0] = -angleTemp[0];
						}
						else
							angles[0] = angleTemp[0];
						thrdataNao << "angles[0]" << angles[0] * 57.296 << ",";

						angleTemp[1] = getAngle(getDiff(coordnateOfNao[5], coordnateOfNao[4]), setToZero(getDiff(coordnateOfNao[5], coordnateOfNao[4]), 'y'));
						//cout << "angleTemp[1]=" << angleTemp[1] << endl;
						if (getDiff(coordnateOfNao[5], coordnateOfNao[4]).y > 0.0)
							angles[1] = -angleTemp[1];
						else
							angles[1] = angleTemp[1];
						thrdataNao << "angles[1] = " << angles[1] * 57.296 << ",";

						angleTemp[2] = getAngle(getDiff(coordnateOfNao[6], coordnateOfNao[5]), getDiff(coordnateOfNao[5], coordnateOfNao[4]));
						if (angleTemp[2] > 1.5708)
							angles[2] = -1.5708;
						else
							angles[2] = -angleTemp[2];
						thrdataNao << "angles[2] = " << angles[2] * 57.296 << ",";

						angleTemp[3] = getAngle(getDiff(coordnateOfNao[6], coordnateOfNao[4]), setToZero(getDiff(coordnateOfNao[6], coordnateOfNao[4]), 'y'));
						if (getDiff(coordnateOfNao[6], coordnateOfNao[5]).y > 0.0)
							angles[3] = angleTemp[3];
						else
							angles[3] = -angleTemp[3];
						thrdataNao << "angles[3] = " << angles[3] * 57.296 << endl;


						angleTemp[4] = getAngle(getDiff(coordnateOfNao[9], coordnateOfNao[8]), setToZero(getDiff(coordnateOfNao[9], coordnateOfNao[8]), 'z'));
						if (getDiff(coordnateOfNao[9], coordnateOfNao[8]).z > 0.0)
						{
							angles[4] = -angleTemp[4];
						}
						else
							angles[4] = angleTemp[4];
						thrdataNao << "angles[4] = " << angles[4] * 57.296 << ",";


						angleTemp[5] = getAngle(getDiff(coordnateOfNao[9], coordnateOfNao[8]), setToZero(getDiff(coordnateOfNao[9], coordnateOfNao[8]), 'y'));

						if (getDiff(coordnateOfNao[9], coordnateOfNao[8]).y > 0.0)
							angles[5] = -angleTemp[5];
						else
							angles[5] = angleTemp[5];
						thrdataNao << "angles[5] = " << angles[5] * 57.296 << ",";

						angleTemp[6] = getAngle(getDiff(coordnateOfNao[10], coordnateOfNao[9]), getDiff(coordnateOfNao[9], coordnateOfNao[8]));
						if (angleTemp[2] > 1.5708)
							angles[6] = 1.5708;
						else
							angles[6] = angleTemp[6];
						thrdataNao << "angles[6] = " << angles[6] * 57.296 << ",";

						angleTemp[7] = getAngle(getDiff(coordnateOfNao[10], coordnateOfNao[9]), setToZero(getDiff(coordnateOfNao[10], coordnateOfNao[8]), 'y'));
						if (getDiff(coordnateOfNao[10], coordnateOfNao[8]).y > 0.0)
							angles[7] = angleTemp[7];
						else
							angles[7] = -angleTemp[7];
						thrdataNao << "angles[7] = " << angles[7] * 57.296 << ",";

						motion.call<void>("angleInterpolation", "LShoulderPitch", angles[0], 1.0f, true);
						motion.call<void>("angleInterpolation", "RShoulderPitch", angles[4], 1.0f, true);

						motion.call<void>("angleInterpolation", "LShoulderRoll",angles[1], 1.0f, true);
						motion.call<void>("angleInterpolation", "RShoulderRoll", angles[5], 1.0f, true);
						
						motion.call<void>("angleInterpolation", "LElbowRoll", angles[2], 1.0f, true);
						motion.call<void>("angleInterpolation", "RElbowRoll", angles[6], 1.0f, true);

						motion.call<void>("angleInterpolation", "LElbowYaw", angles[3], 1.0f, true);
						motion.call<void>("angleInterpolation", "RElbowYaw", angles[7], 1.0f, true);
					}
				}
				imshow("skeletonImage", skeletonImage); //��ʾͼ��   
			}
			else
			{
				cout << "Buffer length of received texture is bogus\r\n" << endl;
			}

			if (cvWaitKey(20) == 27)
				break;
			t++;
			//lastT = now;
			//cout << "t=" << t << endl;
		//}
		if (t > 40)
			break;
	}
	//posture.call<bool>("goToPosture", "Crouch", 0.8f);
	motion.call<void>("rest");
	app.run();
	thrdataKinect.close();
	thrdataNao.close();
	//�ر�NUI����
	NuiShutdown();

}


void drawSkeleton(Mat &image, CvPoint pointSet[], int whichone)
{
	CvScalar color;
	switch (whichone) //���ٲ�ͬ������ʾ��ͬ����ɫ   
	{
	case 0:
		color = cvScalar(255);
		break;
	case 1:
		color = cvScalar(0, 255);
		break;
	case 2:
		color = cvScalar(0, 0, 255);
		break;
	case 3:
		color = cvScalar(255, 255, 0);
		break;
	case 4:
		color = cvScalar(255, 0, 255);
		break;
	case 5:
		color = cvScalar(0, 255, 255);
		break;
	}

	if ((pointSet[NUI_SKELETON_POSITION_HEAD].x != 0 || pointSet[NUI_SKELETON_POSITION_HEAD].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_HEAD], pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_SPINE].x != 0 || pointSet[NUI_SKELETON_POSITION_SPINE].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SPINE], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_SPINE].x != 0 || pointSet[NUI_SKELETON_POSITION_SPINE].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_SPINE], pointSet[NUI_SKELETON_POSITION_HIP_CENTER], color, 2);

	//����֫   
	if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_HAND_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_HAND_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], pointSet[NUI_SKELETON_POSITION_HAND_LEFT], color, 2);

	//����֫   
	if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], pointSet[NUI_SKELETON_POSITION_HAND_RIGHT], color, 2);

	//����֫   
	if ((pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_HIP_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_HIP_CENTER], pointSet[NUI_SKELETON_POSITION_HIP_LEFT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_HIP_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_LEFT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_HIP_LEFT], pointSet[NUI_SKELETON_POSITION_KNEE_LEFT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_KNEE_LEFT], pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_FOOT_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_FOOT_LEFT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT], pointSet[NUI_SKELETON_POSITION_FOOT_LEFT], color, 2);

	//����֫   
	if ((pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_HIP_CENTER], pointSet[NUI_SKELETON_POSITION_HIP_RIGHT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_HIP_RIGHT], pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT], pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT], color, 2);
	if ((pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].y != 0) &&
		(pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT].y != 0))
		line(image, pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT], pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT], color, 2);
}

Vector4 getDiff(Vector4 &vector1, Vector4 &vector2)
{
	Vector4 temp;
	temp.x = vector1.x - vector2.x;
	temp.y = vector1.y - vector2.y;
	temp.z = vector1.z - vector2.z;
	temp.w = vector1.w - vector2.w;
	return temp;
}

double getAngle(Vector4 &vector1, Vector4 &vector2)
{
	double x, y, z, a, b;
	double angle;
	x = vector1.x*vector2.x;
	y = vector1.y*vector2.y;
	z = vector1.z*vector2.z;
	a = vector1.x*vector1.x + vector1.y*vector1.y + vector1.z*vector1.z;
	b = vector2.x*vector2.x + vector2.y*vector2.y + vector2.z*vector2.z;
	angle = acos((x + y + z) / (sqrt(a)*sqrt(b)));
	return angle;

}
Vector4 setToZero(Vector4 &vector, char m)
{
	Vector4 temp;
	if (m == 'x')
	{
		temp.x = 0.0;
		temp.y = vector.y;
		temp.z = vector.z;
		temp.w = vector.w;
	}

	else if (m == 'y')
	{
		temp.x = vector.x;
		temp.y = 0.0;
		temp.z = vector.z;
		temp.w = vector.w;
	}

	else if (m == 'z')
	{
		temp.x = vector.x;
		temp.y = vector.y;
		temp.z = 0.0;
		temp.w = vector.w;
	}
	return temp;
}

