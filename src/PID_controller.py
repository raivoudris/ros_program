#!/usr/bin/env python3

#-------------------------------------- PID KONTROLLERI ARVUTUSED ----------------------------------------------#
def PID(position, delta_t):
    prev_e = 0
    In_al = 0
    PID = 0
    #Joone järgimine
    Kp = 0.04
    Ki = 0.01     
    Kd = 0.3    #Vähendab roboti ujumist 


    #Mida vaja:
    #delta_t, position

    #P
    
    
    if delta_t == 0:
        print("Waiting for delta_t")
    else:
        error = 4.5 - position
        #I - integral
            
        In_al = In_al + (delta_t * error ) #in_al = integral
        #print("integral: ", self.In_al)
        #anti-windup - väldib integraalvea liigset suurenemist
        In_al = max(min(In_al, 1.8), -1.8)
        #print("integral + anti-windup: ", self.In_al)
        #D - derivative
        #print (delta_t)
        err_der = (error - prev_e) / (delta_t * 25)
        err_der = max(min(err_der, 2), -2)
        '''
        print("P ", error)
        print("I ", In_al)
        
        print("D ", err_der)
        '''

    
        Kpe = Kp * error
        Kie = Ki * In_al
        Kde = Kd * err_der
        print("Kie: ", Kie)
        #print(round(Kpe, 2), round(Kie, 2), round(Kde, 2))
        PID = Kpe + Kie + Kde
        prev_e = error
    
    #print("PID FAILIS: ", self.PID)
    
    return PID