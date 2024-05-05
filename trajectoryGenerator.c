@ -1,145 +1,197 @@
#include <math.h>
#include <stdlib.h>
#include "trajectoryGenerator.h"
#include "timer.h"
#include "Robot.h"
#include "utilities.h"
#include "UART_Protocol.h"
#include "QEI.h"


volatile GhostPosition ghostPosition;
static unsigned long lastUpdateTime = 0;
double thetaRobot;
int newPos = 0;

double maxAngularSpeed = 2*PI;
double angularAccel = 2*PI;

void InitTrajectoryGenerator(void) {
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0.0;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.angleToTarget = 0.0;
    ghostPosition.distanceToTarget = 0.0;
    ghostPosition.state = IDLE;
}

void UpdateTrajectory() // Mise a jour de la trajectoire en fonction de l'etat actuel
{
    ///Calcul de quelques variables interm��diaires qui pourraient servir...
    // Angle de la cible
    double targetAngle = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double angleAParcourir = ModuloByAngle(ghostPosition.theta, targetAngle - ghostPosition.theta);
    double angleArret = ghostPosition.angularSpeed*ghostPosition.angularSpeed/(2*angularAccel);
        
    if(angleAParcourir != 0) ///On doit tourner
    {
        //Soit l'angle � parcourir est positif 
        if(angleAParcourir>0)
        {
            //Soit l'angle � parcourir est sup�rieur � la distance d'arr�t
            if(angleAParcourir > angleArret)
            {
                //Soit on a d�j� atteint la Vmax angulaire
                if(ghostPosition.angularSpeed >= maxAngularSpeed)
                {
                    //On maintient la vitesse
                }
                else
                {
                    //Soit on ne l'a pas atteint
                    //On acc�l�re avec saturation � VMax
                    ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel /FREQ_ECH_QEI, maxAngularSpeed);
                }
            }
            //Soit l'angle � parcourir est inf�rieur
            else                
            {
                //On freine
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel /FREQ_ECH_QEI, 0);               
            }            
        }       
        
        //Soit l'angle � parcourir est n�gatif 
        else
        {
            //Soit l'angle � parcourir est sup�rieur � la distance d'arr�t
            if(abs(angleAParcourir) > angleArret)
            {
                //Soit on a d�j� atteint la Vmax angulaire
                if(ghostPosition.angularSpeed <= -maxAngularSpeed)
                {
                    //On maintient la vitesse
                }
                else
                {
                    //Soit on ne l'a pas atteint
                    //On acc�l�re avec saturation � VMax
                    ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel /FREQ_ECH_QEI, -maxAngularSpeed);
                }
            }
            //Soit l'angle � parcourir est inf�rieur
            else                
            {
                //On freine
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel /FREQ_ECH_QEI, 0);               
            }     
        }
        
        ghostPosition.theta += ghostPosition.angularSpeed / FREQ_ECH_QEI;
        //Si la nouvelle vitesse angulaire est nulle ici
        //On a termin� la rotation, l'angle du ghost est donc l'angle de la cible (on casse les erreurs d'arrondi d'int�gration)
        if(ghostPosition.angularSpeed==0)
        {
            ghostPosition.theta = targetAngle;
        }
    }
    robotState.consigneLin = ghostPosition.linearSpeed;
    robotState.consigneAng = ghostPosition.angularSpeed;
}

void SendGhostData() {
    unsigned char ghostPayload[16];
    getBytesFromFloat(ghostPayload, 0, (float) ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 4, (float) ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 8, (float) ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 12, (float) ghostPosition.angularSpeed);
    
    UartEncodeAndSendMessage(GHOST_DATA, 16, ghostPayload);
}