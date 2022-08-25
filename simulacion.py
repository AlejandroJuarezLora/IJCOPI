import importlib
import numpy as np
import nengo

import segway; importlib.reload(segway)


def compute_coeficients(ti, tf, qi, qf, dqi, dqf):
    g = np.matrix([[qi], [qf], [dqi], [dqf]])
    ti2 = ti * ti
    ti3 = ti2 * ti
    tf2 = tf * tf
    tf3 = tf2 * tf
    T = np.matrix([  [1, ti,     ti2,            ti3],
                    [1, tf,     tf2,            tf3],
                    [0,  1,     2 * ti,     3 * ti2],
                    [0,  1,     2 * tf2,    3 * tf3]])
    a = np.dot(np.linalg.inv(T),g)
    return a

tfin = 10
qw_i = 4.
qw_f = 6.
dq_if = 0.

qb_i = 0.1
qb_f = 0.
estado_inicial = np.array([qw_i,dq_if, qb_i, dq_if])  #condiciones iniciales q
a_qw  = compute_coeficients(0,tfin, qw_i, qw_f, dq_if,dq_if)  #generando polinomio para trayectoria deseada de qw
a_qb = compute_coeficients(0,tfin, qb_i, qb_f, dq_if,dq_if)  #generando polinomio para trayectoria deseada de qb


def generate():
    

    segway_sim = segway.SegwayLink(dt = 1e-4)
    segway_sim.init_q = estado_inicial
    segway_sim.reset()

    model = nengo.Network(seed=0)
    with model:
        #creando el nodo del segway
        model.segway_node = segway_sim.create_nengo_node()
        #creando la referencia
        model.ref = nengo.Network('Referencia')
        with model.ref:
            def path_desired(t):
                if (t>tfin):
                    t = tfin
                t2 = t ** 2
                t3 = t ** 3
                t2 = t ** 2
                pos = np.matrix([1,t,t2, t3])
                vel = np.matrix([0,1,2 * t, 3 * t2])
                qw_d = pos * a_qw
                dqw_d = vel * a_qw
                qb_d = pos * a_qb
                dqb_d = vel * a_qb
                return np.array([qw_d[0,0], dqw_d[0,0],  qb_d[0,0], dqb_d[0,0]])
            model.ref.output = nengo.Node(output = path_desired, label = 'Referencia')

        model.error_q0 = nengo.Ensemble(100,1,radius = 3.1416,label = 'Error q0')
        model.error_dq0 = nengo.Ensemble(100,1,radius = 3.1416)
        model.error_q1 = nengo.Ensemble(100, 1, radius = 3.1416)
        model.error_dq1 = nengo.Ensemble(100,1,radius = 3.1416)

        #enviando la referencia al nodo segway para graficarla
        nengo.Connection(model.ref.output[0], model.segway_node[1])
        nengo.Connection(model.ref.output[2], model.segway_node[2])

        #envia el estado deseado al nodo error (transform = -1 es para restar x - xd)
        nengo.Connection(model.ref.output[0], model.error_q0, transform = -1)
        nengo.Connection(model.ref.output[1], model.error_dq0, transform = -1)
        nengo.Connection(model.ref.output[2], model.error_q1, transform = -1)
        nengo.Connection(model.ref.output[3], model.error_dq1, transform = -1)
        
        #creando un ensamble que mapee los estados del segway a la red
        model.cerebelo = nengo.Ensemble(1000, 4, radius = 10)

        #conectando la salida del segway al cerebelo
        nengo.Connection(model.segway_node, model.cerebelo)

        #conectando la salida del segway al error
        nengo.Connection(model.cerebelo[0], model.error_q0)
        nengo.Connection(model.cerebelo[1], model.error_dq0)
        nengo.Connection(model.cerebelo[2], model.error_q1)
        nengo.Connection(model.cerebelo[3], model.error_dq1)

        #creando nodos de control individuales
        model.u_q0 = nengo.Ensemble(1000,1, radius = 350)
        model.u_dq0 = nengo.Ensemble(1000,1, radius = 250)
        model.u_q1 = nengo.Ensemble(1000,1, radius = 180)
        model.u_dq1 = nengo.Ensemble(1000,1, radius = 250)
        
        nengo.Connection(model.error_q0, model.u_q0, transform = 100)
        nengo.Connection(model.error_dq0, model.u_dq0, transform = 323.3434)
        nengo.Connection(model.error_q1, model.u_q1, transform = 542.0927)
        nengo.Connection(model.error_dq1, model.u_dq1, transform = 541.08)
       
        #Enviando el control al segway
        nengo.Connection(model.u_q0, model.segway_node[0])
        nengo.Connection(model.u_dq0, model.segway_node[0])
        nengo.Connection(model.u_q1, model.segway_node[0])
        nengo.Connection(model.u_dq1, model.segway_node[0])

    return model

# Check to see if it's open in the GUI
from nengo.simulator import Simulator as NengoSimulator
if nengo.Simulator is not NengoSimulator or __name__ == '__main__':
    # connect up the models we've defined, set up the functions, probes, etc
    model = generate()


