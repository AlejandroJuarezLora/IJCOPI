import numpy as np
import nengo

class SegwayLink:
    def __init__(self, init_q = [0.,0.,0., 0.], dt = 1e-4):
        self.DOF = 2
        self.dt = dt
        self.init_q = np.copy(init_q)

        mb = 1
        mw = 2
        r = 0.25
        L = 1.2
        Iw = 10
        Ib = 10
        g = 9.81

        self.r = r
        self.L = L

        self.m11 = (mb + mw) * r**2 + Iw
        self.m12 = mb * L * r 
        self.m21 = self.m12
        self.m22 = mb * L**2 + Ib
        self.f1 = mb * L  * r
        self.f2 = mb * g * L

        self.reset()

    def aplica_torque(self, u, dt = None):
        """modelo del segway"""
        dt = self.dt if dt is None else dt
        q0, dq0, q1, dq1 = self.q

        cosq1 = np.cos(q1)
        sinq1 = np.sin(q1) 
        
        m11 = self.m11
        m12 = self.m12 * cosq1
        m21 = m12
        m22 = self.m22
        f1 = u + self.f1 * cosq1
        f2 = -u + self.f2 * sinq1

        detM = (m11*m22 - m12*m21)
        ddq0 = (f1*m22)/detM - (f2*m12)/detM
        ddq1 = (f2*m11)/detM - (f1*m21)/detM
        
        dq0 = dq0 + dt * ddq0
        dq1 = dq1 + dt * ddq1

        q0 = q0 + dt * dq0
        q1 = q1 + dt + dq1

        return [q0, dq0, q1, dq1]

    def planta_linealizada(self, u, dt = None):
        """modelo del segway linealizado, devuelve [q0, dq0, q1, dq1]"""
        dt = self.dt if dt is None else dt
        mb = 1
        mw = 2
        r = 0.25
        L = 1.2
        Iw = 10
        Ib = 10
        g = 9.81

        dt = self.dt if dt is None else dt
        x1, x2, x3, x4 = self.q

        npsinx3 = np.sin(x3)
        npcosx3 = np.cos(x3)

        dx1 = x2
        dx2 = (r * npsinx3*L**3*mb**2*x4**2 - g*r*npcosx3*npsinx3*L**2*mb**2 + u*L**2*mb + Ib*r*npsinx3*L*mb*x4**2 + r*u*npcosx3*L*mb + Ib*u)/(- L**2*mb**2*r**2*npcosx3**2 + L**2*mb**2*r**2 + mw*L**2*mb*r**2 + Iw*L**2*mb + Ib*mb*r**2 + Ib*mw*r**2 + Ib*Iw)
        dx3 = x4
        dx4 =  -(npcosx3*npsinx3*L**2*mb**2*r**2*x4**2 - g*npsinx3*L*mb**2*r**2 - g*mw*npsinx3*L*mb*r**2 + u*npcosx3*L*mb*r - Iw*g*npsinx3*L*mb + u*mb*r**2 + mw*u*r**2 + Iw*u)/(- L**2*mb**2*r**2*npcosx3**2 + L**2*mb**2*r**2 + mw*L**2*mb*r**2 + Iw*L**2*mb + Ib*mb*r**2 + Ib*mw*r**2 + Ib*Iw)

        x1 = x1 + dx1 * dt
        x2 = x2 + dx2 * dt
        x3 = x3 + dx3 * dt
        x4 = x4 + dx4 * dt
        
        return [x1,x2,x3,x4]


    def reset(self, q=None):
        """Resetea el estado inicial del segway
        """
        self.q = self.init_q if q is None else q

    def posicion(self, q = None):
        """ Calcula la posicion actual del centro de la rueda y del el brazo """
        q0, dq0, q1, dq1 = self.q if q is None else q

        xw = self.r * q0
        yw = self.r
        xb = self.L * np.sin(q1) + self.r * q0
        yb = self.r + self.L * np.cos(q1)

        return [xw, yw, xb, yb]

    
    def create_nengo_node(self):
        def segway_func(t, u):
            #la funcion es la salida a nengo
            #devuelve self.q, self.dq
            self.q=  self.planta_linealizada(u[0])
            #codigo para la animacion
            q0r = u[1]      #trayendo la referencia
            q1r = u[2]      #trayendo la referencia
            [q0,dq0, q1, dq1] = self.q
            escala = 30
            l = self.L * escala
            r = self.r * escala
            x_offset = 50
            #obteniendo las coordenadas cartesianas
            
            xwreal, ywreal, xbreal, ybreal = self.posicion(self.q)
           #obteniendo las coordenadas cartesianas escaladas
            xw =r * float(q0)
            yw = r 
            xb = xw  + l * np.sin(float(q1)) 
            yb = yw + l * np.cos(float(q1)) 

            #obteniendo las coordenadas cartesianas de la referencia
            xwr =r * float(q0r)
            ywr = r 
            xbr = xwr  + l * np.sin(float(q1r)) 
            ybr = ywr + l * np.cos(float(q1r))

            #animando
            segway_func._nengo_html_ = '''
             <svg width="100%" height="100%" viewbox="0 0 100 100">
                <text x="0" y="15" style="font-size:5px">xw:{xwreal}, yw:{ywreal}</text>
                <text x="0" y="20" style="font-size:5px">xb:{xbreal}, yb:{ybreal}</text>
                <g id="cartesian" transform="translate(0,100) scale(1,-1)">
                <line x1="{xw}" y1="{yw}"
                x2="{xb}" y2="{yb}" style="stroke:black"/>
                <circle cx="{xw}" cy="{yw}"
                r="{r}" stroke="black" stroke-width="1" fill="none" />
                <line x1="{xwr}" y1="{ywr}"
                x2="{xbr}" y2="{ybr}" style="stroke:red" stroke-dasharray="2,2" stroke-width="0.5"/>
                <circle cx="{xwr}" cy="{ywr}"
                r="{r}" stroke="red" stroke-width="0.5" stroke-dasharray="2,2" fill="none"/>
                </g>
            </svg>
            '''.format(**locals())

            # #datos devueltos del nodo segway al modelo
            # [a, b, c, d] = self.dq
            # data = np.hstack([q0,dq0, q1, dq1,a, b, c, d])
            return self.q    #simulacion1.py
        return nengo.Node(output = segway_func, size_in = 3, label = 'Animacion Segway')