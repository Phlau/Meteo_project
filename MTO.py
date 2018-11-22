import AC_design as ac
import numpy as np


"""On definit la mission"""
mis = ac.Mission()
mis.add_climb(1500) #On monte de 1500m
mis.add_cruise(2000) #On fait 1000m aller-retour
mis.add_descent(700) #On descend de 700m pour arriver a 800m
mis.add_cruise(2000) #On refait 1000m aller-retour
mis.add_descent(800) #On descend de 800m pour atterrir

"""On definit les constantes"""
W_system_cst = 0.1 * ac.gee
W_payload_cst = 0.15 * ac.gee
P_system_cst = 5.5
P_payload_cst = 2.2
P_max_cst = 100

rho = 1.225

i =0

for V_test in np.linspace(10,30,10):
    for ROC_test in np.linspace(1,5,10):
        for b_test in np.linspace(0.2,1.2,10):
            for S_test in np.linspace(0.01,0.6,10):
                for cap_batt_test in np.linspace(10,40,10):
                    drone = ac.Aircraft(
                        W_system = W_system_cst,
                        W_payload = W_payload_cst,
                        P_system = P_system_cst,
                        P_payload = P_payload_cst,
                        P_max = P_max_cst,
                        b = b_test,
                        S = S_test,
                        cap_batt = cap_batt_test
                    )
                    try:
                        drone.calc_perfo_basic(V_test,ROC_test,rho)
                        mis.set_ac(drone)
                        Vs = np.sqrt(2*drone.W_total/(drone.S*1.25*rho))
                        if drone.W_total < 12. and Vs < 12. and Vs > 8. :
                            print(mis.calc_mission(V_test,ROC_test),V_test,drone.ROC,Vs,i)
                            if i == 1584: #On selectionne ici l'indice de l'avion que l'on veut afficher, pour pouvoir trier les valeurs obtenues
                                print(drone)
                            i+=1

                    except ac.InvalidCondition:
                        pass

# Exemple d'utilisation :
# python MTO.py > out.txt
# sort out.txt > out_sorted.txt
