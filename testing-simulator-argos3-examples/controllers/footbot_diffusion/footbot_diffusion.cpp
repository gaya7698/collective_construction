/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <vector>
#include <complex> 
#include <math.h>
#include <iostream>
#include <argos3/core/control_interface/ci_sensor.h>
#include <stdio.h>
#include <argos3/core/utility/math/quaternion.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>

#define PORT 8080
using namespace std;
/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :

   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcPosition(NULL),
   m_pcFloor(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*

    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   
   m_pcPosition = GetSensor  <CCI_PositioningSensor      >("positioning"    );
   

   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/
int square;
CQuaternion orient;
CVector3 Pose;
double Ww, Zz, RTheta2Base, Theta2Goal, RVz, PoseX, PoseY, goalX, goalY;
CRadians Testt;
string RID;

void comms() {
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char* hello = "Hello from client";
    char buffer[1024] = { 0 };
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return;
    }
 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
 
    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return;
    }
 
    if (connect(sock, (struct sockaddr*)&serv_addr,
                sizeof(serv_addr))
        < 0) {
        printf("\nConnection Failed \n");
        return;
    }
    send(sock, hello, strlen(hello), 0);
    printf("Hello message sent\n");
    valread = read(sock, buffer, 1024);
    printf("%s\n", buffer);
    return;
}

void CFootBotDiffusion::ControlStep() {

   RID = GetId();
   
   //comms(); - currently breaks sim
           
   //int square;
   //m_pcFloor = &GetSpace().GetFloorEntity();
   //CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   const CCI_PositioningSensor::SReading& tPosReads = m_pcPosition->GetReading();
   cout<<"\n"<<CVector3(tPosReads.Position)<<"\n";

   // THETA CALCULATIONS 
   orient=CQuaternion(tPosReads.Orientation);
   Ww = orient.GetW();
   Zz = orient.GetZ();
   RTheta2Base=acos(Ww)*2;
   RVz=Zz/sin(acos(Ww));
   if(RVz<0) RTheta2Base=6.283185307-RTheta2Base;
   

   // POSITION CALCULATIONS
   Pose=CVector3(tPosReads.Position);
   PoseX=Pose.GetX();
   PoseY=Pose.GetY();
   Testt=Pose.GetZAngle();

   ofstream fout;
   // by default ios::out mode, automatically deletes
   // the content of file. To append the content, open in ios:app
   // fout.open("sample.txt", ios::app)
   fout.open(RID);
   // Execute  If file successfully opened
   fout << RID << endl;
   fout << PoseX << " " << PoseY << endl;
   fout << RTheta2Base << endl;
  
   fout.close();


   // GOTO FUNCTION
   goalX=-1.25;
   goalY=-1.25;

   /*Theta2Goal=atan2((goalY-PoseY),(goalX-PoseX));
   if(Theta2Goal<0)Theta2Goal+=6.2831853;
   cout<<"\ncTheta = "<<RTheta2Base<< "\ngTheta"<<Theta2Goal;
   cout<<"\n cX = "<<PoseX<<"\n gX = "<<goalX<<"\n cY = "<<PoseY<<"\n gY = "<<goalY;

   if(RTheta2Base>Theta2Goal || abs(RTheta2Base-Theta2Goal)>3.14159){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
   }
   else if(RTheta2Base<Theta2Goal){
      m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
   }
   else if(goalX==PoseX && goalY==PoseY){
      cout<<"\n Arrived!\n";
   }
   else m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);

   if(RTheta2Base<(Theta2Goal+.009) && RTheta2Base>(Theta2Goal-.009)){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }*/
   
   /*CVector2 cPos;
      cPos.Set(m_cFootbots.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               m_cFootbots.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();

   
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      cout<<"angle";
      cout <<cAngle;
      cout <<"accumulator.length";
      cout << cAccumulator.Length() ;
      cout<<"crazy long line";
      cout<<m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle);
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         cout <<cAngle;
         cout <<"\n\n\n\n angle?! \n";
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         
         cout <<cAngle;
         cout <<"\n\n\n\n angle..! \n";
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
