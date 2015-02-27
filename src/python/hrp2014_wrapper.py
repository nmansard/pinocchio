from robot_wrapper import RobotWrapper
import numpy as np

class Hrp2014Wrapper(RobotWrapper):

    def __init__(self,filename):
        RobotWrapper.__init__(self,filename)
        self.q0 = np.matrix( [
        0., 0., 0.648702, 0., 0. , 0., 1.,               # Free flyer
        0., 0., 0., 0.,                                  # Chest and head
        0.261799, 0.17453,  0., -0.523599, 0., 0., 0.1,  # Left arm
        0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,  # Right arm
        0., 0., -0.453786, 0.872665, -0.418879, 0.,      # Left leg
        0., 0., -0.453786, 0.872665, -0.418879, 0.,      # Righ leg
        ] ).T

        self.opCorrespondances = { "lh": "LARM_JOINT5",
                                   "rh": "RARM_JOINT5",
                                   "lf": "LLEG_JOINT5",
                                   "rf": "RLEG_JOINT5",
                                   }

        for op,name in self.opCorrespondances.items():
            idx = self.__dict__[op] = self.index(name)
            #self.__dict__['_M'+op] = types.MethodType(lambda s,q: s.position(q,idx),self)

    # --- SHORTCUTS ---
    def Mrh(self,q):
        return self.position(q,self.rh)
    def Jrh(self,q):
        return self.jacobian(q,self.rh)
    def wJrh(self,q):
        return se3.jacobian(self.model,self.data,self.rh,q,False)
    def vrh(self,q,v):
        return self.velocity(q,v,self.rh)

    def Jlh(self,q):
        return self.jacobian(q,self.lh)
    def Mlh(self,q):
        return self.position(q,self.lh)

    def Jlf(self,q):
        return self.jacobian(q,self.lf)
    def Mlf(self,q):
        return self.position(q,self.lf)

    def Jrf(self,q):
        return self.jacobian(q,self.rf)
    def Mrf(self,q):
        return self.position(q,self.rf)


__all__ = [ 'Hrp2014Wrapper' ]
