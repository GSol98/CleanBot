import numpy as np
import math as mt
from Plot import plot_slam

class SLAM:

    #====================================
    #       Inizialize landmark
    #====================================
    #
    def __init__(self):
        self.mu = np.zeros((3, 1))
        self.sigma = np.zeros((3,3))

        self.errorMovementX = 0.001
        self.errorMovementY = 0.001
        self.errorMovementTeta = 0.0001

        self.errorMeasurementD = 0.1
        self.errorMeasurementTeta = 0.1

        self.R = [self.errorMovementX**2, self.errorMovementY**2, self.errorMovementTeta**2]
        self.Q = np.array([[self.errorMeasurementD**2, 0], 
                            [0, self.errorMeasurementTeta**2]])


    #====================================
    #         Prediction landmark
    #====================================
    #
    # motion[0] = velocità lineare
    # motion[1] = velocità angolare
    # motion[2] = tempo
    #
    def predictionMove(self, motion):
        self.mu[0,0] += motion[0] * motion[2] * np.cos(self.mu[2,0])
        self.mu[1,0] += motion[0] * motion[2] * np.sin(self.mu[2,0])
        self.mu[2,0] += motion[1] * motion[2]
        while self.mu[2,0] >  np.pi: self.mu[2,0] -= 2*np.pi
        while self.mu[2,0] < -np.pi: self.mu[2,0] += 2*np.pi

        g = np.array([ [1, 0, -motion[0]*motion[2]*np.sin(self.mu[2,0])], [0, 1, motion[0]*motion[2]*np.cos(self.mu[2,0])], [0, 0, 1] ])
        G = np.vstack(( np.hstack(( g , np.zeros((3,len(self.mu)-3)) )) , np.hstack(( np.zeros((len(self.mu)-3,3)) , np.eye(len(self.mu)-3) )) ))

        self.sigma = np.dot( np.dot( G , self.sigma ) , np.transpose(G) )
        self.sigma[0,0] += self.R[0]
        self.sigma[1,1] += self.R[1]
        self.sigma[2,2] += self.R[2]


    #====================================
    #        Control landmark
    #====================================
    #
    # Lista di vettori contenente
    # le misure dei landmark 
    # [[distanza, angolo], ...]
    #
    def controlLandmark(self, measures):
        for measure in measures:
            exist = False
            
            t = np.array([ [-measure[0,0]*np.sin(measure[1,0]+self.mu[2,0])] , [measure[0,0]*np.cos(measure[1,0]+self.mu[2,0])] ])
            Tmu = np.hstack(( np.eye(2) , t ))
            Tz = np.hstack(( np.array([ [np.cos(measure[1,0]+self.mu[2,0])] , [np.sin(measure[1,0]+self.mu[2,0])] ]) , t ))

            sigmaLL = np.dot( np.dot( Tmu , self.sigma[0:3,0:3] ) , np.transpose( Tmu ) )
            sigmaLL += np.dot( np.dot( Tz , self.Q ) , np.transpose( Tz ) )
            sigmaLL = 3 * np.array([np.sqrt(sigmaLL[0,0]), np.sqrt(sigmaLL[1, 1])])

            landmark = np.array([ self.mu[0] + measure[0,0]*np.cos(measure[1,0]+self.mu[2,0]) , self.mu[1] + measure[0,0]*np.sin(measure[1,0]+self.mu[2,0]) ])

            for i in range(2, int( ((len(self.mu)-1) / 2) + 1)):
                if -sigmaLL[0] <= landmark[0] - self.mu[2*i-1,0] <= sigmaLL[0] and -sigmaLL[1] <= landmark[1] - self.mu[2*i,0] <= sigmaLL[1]:
                    t = [i, measure[0,0], measure[1,0]]
                    self.correctionLandmark(t)
                    exist = True
                    break

            if not exist:
                self.addLandmark(landmark, measure)


    #====================================
    #         Add landmark
    #====================================
    #
    # landmark[0] = x       measure[0] = distanza misurata      
    # landmark[1] = y       measure[1] = angolo misurato
    #
    def addLandmark(self, landmark, measure):
        self.mu = np.vstack(( self.mu , landmark ))

        t = np.array([ [-measure[0,0]*np.sin(measure[1,0]+self.mu[2,0])] , [measure[0,0]*np.cos(measure[1,0]+self.mu[2,0])] ])
        Tmu = np.hstack(( np.eye(2) , t ))
        Tz = np.hstack(( np.array([ [np.cos(measure[1,0]+self.mu[2,0])] , [np.sin(measure[1,0]+self.mu[2,0])] ]) , t ))

        sigmaLL = np.dot( np.dot( Tmu , self.sigma[0:3,0:3] ) , np.transpose( Tmu ) )
        sigmaLL += np.dot( np.dot( Tz , self.Q ) , np.transpose( Tz ) )
        sigmaLX = np.dot( Tmu , self.sigma[0:3,:] )

        self.sigma = np.hstack((self.sigma , np.transpose(sigmaLX) ))
        self.sigma = np.vstack((self.sigma , np.hstack(( sigmaLX , sigmaLL ))))


    #====================================
    #         Correction landmark
    #====================================
    #
    # landmark[0] = indice del landmark
    # landmark[1] = distanza del landmark
    # landmark[2] = angolo del landmark
    #
    def correctionLandmark(self, landmark):
        delta = np.array([[ self.mu[2*landmark[0]-1,0] - self.mu[0,0] , self.mu[2*landmark[0],0] - self.mu[1,0] ]]).T
        q = np.sqrt( delta.T.dot(delta) )[0,0]
        h = np.array([[ q , mt.atan2( delta[1,0], delta[0,0] ) - self.mu[2,0] ]]).T
        while h[1,0] >  np.pi: h[1,0] -= 2*np.pi
        while h[1,0] < -np.pi: h[1,0] += 2*np.pi

        H_t1 = np.hstack(( np.array([-q*delta[0,0] , -q*delta[1,0] , 0]) , np.arange(0,2*landmark[0]-4,1)*0 , np.array([q*delta[0,0] , q*delta[1,0]]) , np.arange(0,len(self.mu)-2*landmark[0]-1,1)*0 ))
        H_t2 = np.hstack(( np.array([delta[1,0] , -delta[0,0] , -(q**2)]) , np.arange(0,2*landmark[0]-4,1)*0 , np.array([-delta[1,0] , delta[0,0]]) , np.arange(0,len(self.mu)-2*landmark[0]-1,1)*0 ))
        H  = (1 / (q**2))*np.vstack(( H_t1, H_t2 ))

        K = np.dot( np.dot(self.sigma, H.T) , np.linalg.inv( np.dot( np.dot( H, self.sigma ) , H.T ) + self.Q ))

        T = -np.dot( K , H )
        for i in range(0, len(self.mu)): T[i,i] += 1

        self.mu += np.dot( K , np.array([[landmark[1]], [landmark[2]]]) - h )
        self.sigma = np.dot( T , self.sigma ) 



    #====================================
    #           Utility
    #====================================

    def plotSlam(self):
        plot_slam(self.mu, self.sigma)

    def getMu(self): 
        return self.mu

    def getSigma(self): 
        return self.sigma