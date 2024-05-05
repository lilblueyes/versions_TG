@ -1,197 +1,227 @@
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
int lin = 0;

double maxAngularSpeed = 2 * PI;
double angularAccel = 2 * PI;

double maxLinearSpeed = 1;
double linearAccel = 1;

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
    if ((ghostPosition.targetX != ghostPosition.x || ghostPosition.targetY != ghostPosition.y) && newPos == 0) {
        newPos = 1;
        lin = 0;
    }

    if (newPos) {
        double targetAngle = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
        double angleAParcourir = ModuloByAngle(ghostPosition.theta, targetAngle - ghostPosition.theta);
        double angleArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
        double distanceAParcourir = sqrt((ghostPosition.targetX - ghostPosition.x)*(ghostPosition.targetX - ghostPosition.x)
                +(ghostPosition.targetY - ghostPosition.y)*(ghostPosition.targetY - ghostPosition.y));
        double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);

        if (angleAParcourir != 0) ///On doit tourner
        {
            //Soit l'angle � parcourir est positif 
            if (angleAParcourir > 0) {
                //Soit l'angle � parcourir est sup�rieur � la distance d'arr�t
                if (angleAParcourir > angleArret) {
                    //Soit on a d�j� atteint la Vmax angulaire
                    if (ghostPosition.angularSpeed >= maxAngularSpeed) {
                        //On maintient la vitesse
                    } else {
                        //Soit on ne l'a pas atteint
                        //On acc�l�re avec saturation � VMax
                        ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
                    }
                }//Soit l'angle � parcourir est inf�rieur
                else {
                    //On freine
                    ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
                }
            }//Soit l'angle � parcourir est n�gatif 
            else {
                //Soit l'angle � parcourir est sup�rieur � la distance d'arr�t
                if (abs(angleAParcourir) > angleArret) {
                    //Soit on a d�j� atteint la Vmax angulaire
                    if (ghostPosition.angularSpeed <= -maxAngularSpeed) {
                        //On maintient la vitesse
                    } else {
                        //Soit on ne l'a pas atteint
                        //On acc�l�re avec saturation � VMax
                        ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, -maxAngularSpeed);
                    }
                }//Soit l'angle � parcourir est inf�rieur
                else {
                    //On freine
                    ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
                }
            }

            ghostPosition.theta += ghostPosition.angularSpeed / FREQ_ECH_QEI;
            //Si la nouvelle vitesse angulaire est nulle ici
            //On a termin� la rotation, l'angle du ghost est donc l'angle de la cible (on casse les erreurs d'arrondi d'int�gration)
            if (ghostPosition.angularSpeed == 0) {
                ghostPosition.theta = targetAngle;
            }
        } else if (distanceAParcourir != 0) {
            if (lin == 0 && ghostPosition.linearSpeed > 0.075) {
                lin = 1;
            }
            //Soit la distance � parcourir est sup�rieure � la distance d'arret
            if (distanceAParcourir > (distanceArret + ghostPosition.linearSpeed / FREQ_ECH_QEI)) // Savoir si 
            {
                //Soit on a d�j� atteint la Vmax
                if (ghostPosition.linearSpeed >= maxLinearSpeed) {
                    //On fait rien
                } else {
                    //On acc�l�re (en saturant � Vmax)
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, maxLinearSpeed);
                }
            }//Soit la distane � parcourir est inf�rieure � la distance d'arret
            else {
                //On freine
                ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
            }

            double distanceParcourue = ghostPosition.linearSpeed / FREQ_ECH_QEI;
            ghostPosition.x += distanceParcourue * cos(ghostPosition.theta);
            ghostPosition.y += distanceParcourue * sin(ghostPosition.theta);

            //Si la nouvelle vitesse lin�aire est nulle ici
            //On a termin� le parcours, la position du ghost est donc la position de la cible (on casse les erreurs d'arrondi d'int�gration)
            if (ghostPosition.linearSpeed < 0.075 && lin == 1) {
                ghostPosition.x = ghostPosition.targetX;
                ghostPosition.y = ghostPosition.targetY;
                ghostPosition.linearSpeed = 0;
                newPos = 0;
            }
        }

        robotState.consigneLin = ghostPosition.linearSpeed;
        robotState.consigneAng = ghostPosition.angularSpeed;
    }
}

void SendGhostData() {
    unsigned char ghostPayload[16];
    getBytesFromFloat(ghostPayload, 0, (float) ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 4, (float) ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 8, (float) ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 12, (float) ghostPosition.linearSpeed);

    UartEncodeAndSendMessage(GHOST_DATA, 16, ghostPayload);
}