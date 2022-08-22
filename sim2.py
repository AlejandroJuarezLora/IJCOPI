import importlib
import numpy as np
import nengo

import segway; importlib.reload(segway)


def generate():
    estado_inicial = np.array([4.,0., 0.2, 0.])  #condiciones iniciales q
    estado_deseado = np.array([6., 0., 0., 0.])
    estado_deseado2 = np.array([4., 0., 0., 0.])

    segway_sim = segway.SegwayLink(dt = 1e-3)
    segway_sim.init_q = estado_inicial
    segway_sim.reset()

    model = nengo.Network(seed=0)
    with model:
        #creando el nodo del segway
        model.segway_node = segway_sim.create_nengo_node()
        #creando la referencia
        model.ref = nengo.Network('REF')
        with model.ref:
            def PMC_func(t):
                if t<15:
                    return estado_deseado
                return estado_deseado2
            model.ref.output = nengo.Node(output = PMC_func, label = 'Referencia')

        #enviando la referencia al nodo segway para graficarla
        nengo.Connection(model.ref.output[0], model.segway_node[1])
        nengo.Connection(model.ref.output[2], model.segway_node[2])
        
        model.error = nengo.Ensemble(300,4, radius = 3.1416, label = 'Error')

        

        #creando un ensamble que mapee los estados del segway a la red
        model.cerebelo = nengo.Ensemble(1000, 4, radius = 10)

         #creando nodo de control
        model.u = nengo.Ensemble(1000,4, radius = 350)

        #envia el estado deseado al nodo error (transform = -1 es para restar x - xd)
        nengo.Connection(model.ref.output, model.error, transform = -1)
        
        #conectando la salida [dq0, ddq0, dq1, ddq1] del segway al cerebelo
        nengo.Connection(model.segway_node[:4], model.cerebelo)
        #conectando la salida del segway al error
        nengo.Connection(model.cerebelo, model.error)

        nengo.Connection(model.error, model.u, transform = [100,323.3434,542.0927,541.08])
        
        # nengo.Connection(model.u[0], model.segway_node)
        # nengo.Connection(model.u[1], model.segway_node)
        # nengo.Connection(model.u[2], model.segway_node)
        # nengo.Connection(model.u[3], model.segway_node)

        model.u_sum = nengo.Ensemble(1000,1,radius=40)
        nengo.Connection(model.u[0], model.u_sum)
        nengo.Connection(model.u[1], model.u_sum)
        nengo.Connection(model.u[2], model.u_sum)
        nengo.Connection(model.u[3], model.u_sum)
        #enviando la señal de control
        nengo.Connection(model.u_sum, model.segway_node[0])
        
        #creando nodo de control aprendido
        model.u_est = nengo.Ensemble(1000,1,radius=40)
        conn = nengo.Connection(model.u_sum, model.u_est, function=lambda x: np.random.random())
        model.error_u = nengo.Ensemble(500, 1, radius = 10)
        nengo.Connection(model.u_sum,model.error_u, transform=-1)
        nengo.Connection(model.u_est, model.error_u)
        # Add the learning rule to the connection
        conn.learning_rule_type = nengo.PES()
    
        # Connect the error into the learning rule
        nengo.Connection(model.error_u, conn.learning_rule)

       
    
    return model
    

# Check to see if it's open in the GUI
from nengo.simulator import Simulator as NengoSimulator
if nengo.Simulator is not NengoSimulator or __name__ == '__main__':
    # connect up the models we've defined, set up the functions, probes, etc
    model = generate()


   

    


