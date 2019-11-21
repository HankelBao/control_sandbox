
class Path(chrono.ChBezierCurve):
    def __init__(self, generator):
        chrono.ChBezierCurve.__init__(self, generator.GetChPath())

        self.path = generator.GetChPath()

    def GetChPath(self):
        return self.path

    def calcClosestIndex(self,loc):
        best_i = 0
        best_len = 1000
        for i in range(self.path.size()-1):
            vec = loc - self.path[i]
            len = vec.Length()
            if len < best_len:
                best_i = i
                best_len = len
        return best_i

    def GetArcLength(self,index):
        len = 0
        for i,j in zip(range(0,self.path.size()-1), range(1,self.path.size())):
            p1 = self.path[i]
            p2 = self.path[j]
            vec = p2 - p1
            len += vec.Length()
            if j >= index:
                return len
        return len


class PathTracker(chrono.ChBezierCurveTracker):
    def __init__(self, path):
        chrono.ChBezierCurveTracker.__init__(self, path, True)
        self.path = path
        self.ch_path = self.path.GetChPath()
        self.starting_index = 1

    def GetInitLoc(self):
        return self.ch_path[self.starting_index]

    def GetInitRot(self):
        y_axis = chrono.ChVectorD(1,0,0)
        vec = self.ch_path[self.starting_index+1] - self.ch_path[self.starting_index]
        theta = math.acos((y_axis^vec)/(vec.Length()*y_axis.Length()))
        q = chrono.ChQuaternionD()
        q.Q_from_AngZ(-theta)
        return q
