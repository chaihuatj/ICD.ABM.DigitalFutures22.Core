using ABxM.Core.Agent;
using ABxM.Core.Behavior;
using ICD.ABM.DigitalFutures22.Core.AgentSystem;
using Rhino.Geometry;
using System;
using System.Collections.Generic;




namespace ICD.ABM.DigitalFutures22.Core.Agent
{
    public class RailAgent : AgentBase
    {
        public Point2d UV = Point2d.Unset;
        public double[] t = new double[2] { double.NaN, double.NaN };
        
        public Point2d startUV;

        public bool isFixed = false;

        public Curve GuideCurve;
        public List<RailAgent> SameGuideCurce = new List<RailAgent>();
        public RailAgent[] Neighbors = new RailAgent[2] {null,null};

        public double CurvatureAccuracy = 20;
        public int alg = 3;
        public bool max = true;
        /// <summary>
        /// The list of 2-dimensional moves
        /// </summary>
        public List<Vector2d> Moves = new List<Vector2d>();
        public List<double> Weights = new List<double>();

        public RailAgent(Point2d uvParameter, List<BehaviorBase> behaviors)
        {
            this.UV = this.startUV = uvParameter;
            this.Behaviors = behaviors;
        }
        
        public override void Reset()
        {
            this.Moves.Clear();
            this.UV = this.startUV;
            this.isFixed = false;
            this.t=new double[2] { double.NaN, double.NaN };

            this.Neighbors = new RailAgent[2] { null, null };
            this.SameGuideCurce.Clear();
        }
        public override void PreExecute()
        {
            this.Moves.Clear();

            this.t = new double[2] { double.NaN, double.NaN };
            this.Neighbors = new RailAgent[2] { null, null };
            this.SameGuideCurce.Clear();

            this.FindGuideCurve();
            this.FindSameGuideCurve();
            this.ComputeT(CurvatureAccuracy, max, 0, alg);
            this.FindNeighbors();

        }

        public override void Execute()
        {


            foreach (BehaviorBase behavior in this.Behaviors)
                behavior.Execute(this);
 
        }

        public override void PostExecute()
        {
            if (this.isFixed == false)
            {
                if (this.Moves.Count > 0)
                {
                    Vector2d totalWeightedMove = Vector2d.Zero;
                    double totalWeight = 0.0;

                    for (int i = 0; i < Moves.Count; ++i)
                    {
                        totalWeightedMove += Weights[i] * Moves[i];
                        totalWeight += Weights[i];
                    }

                    (this.AgentSystem as RailAgentSystem).moveDistances.Add((totalWeightedMove / totalWeight).Length);

                    Point2d p1 = this.UV + totalWeightedMove / totalWeight;
                    Point3d p3d = (this.AgentSystem as RailAgentSystem).SystemSurface.PointAt(p1.X, p1.Y);
                    double ttt;
                    this.GuideCurve.ClosestPoint(p3d, out ttt);
                    Point3d p3dd = this.GuideCurve.PointAt(ttt);
                    double u11, v11;
                    (this.AgentSystem as RailAgentSystem).SystemSurface.ClosestPoint(p3dd, out u11, out v11);


                    this.UV = new Point2d(u11, v11);
                }

            }

        }



        public void FindGuideCurve()
        {
            foreach (Curve guideCurve in (this.AgentSystem as RailAgentSystem).SystemCurves)
            {
                double tPara=new double();
                Point3d position = (this.AgentSystem as RailAgentSystem).SystemSurface.PointAt(this.UV.X, this.UV.Y);
                guideCurve.Domain = new Interval(0.0, 1.0);
                guideCurve.ClosestPoint(position, out tPara);
                Point3d cloPt = guideCurve.PointAt(tPara);

                //if (position.DistanceTo(cloPt) < Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)
                if (position.DistanceTo(cloPt) < 0.01)

                {
                    this.GuideCurve = guideCurve;
                    this.t = new double[2] { double.NaN, double.NaN };
                    this.t[0]=tPara;

                    if (tPara<=0.01 || tPara>=0.99) this.isFixed = true;
                }

            }
        }
        public void FindSameGuideCurve()
        {
            this.SameGuideCurce.Clear();

            foreach (RailAgent agent in (this.AgentSystem as RailAgentSystem).Agents)
            {
                if (agent.UV != this.UV && agent.GuideCurve == this.GuideCurve) this.SameGuideCurce.Add(agent);
            }

        }


        public List<double> ComputeCurveDistance(Curve CurveA, Curve CurveB, out double[] MaxandMin)
        {
            CurveA.Domain = new Interval(0.0, 1.0);
            CurveB.Domain = new Interval(0.0, 1.0);

            List<double> curveT = new List<double>();
            for (int j = 0; j < 200; j++) { curveT.Add((double)j / 199); }


            List<double> disAB = new List<double>();
            List<double> disBA = new List<double>();
            List<double> disABBA = new List<double>();
            foreach (double t in curveT)
            {
                double ta, tb;
                CurveA.ClosestPoint(CurveB.PointAt(t), out ta);
                CurveB.ClosestPoint(CurveA.PointAt(t), out tb);

                disAB.Add(CurveA.PointAt(t).DistanceTo(CurveB.PointAt(tb)));
                disBA.Add(CurveB.PointAt(t).DistanceTo(CurveA.PointAt(ta)));

            }
            disAB.Sort((x, y) => -x.CompareTo(y));
            disBA.Sort((x, y) => -x.CompareTo(y));

            double dmax = disAB[0];
            double dmin = disAB[disAB.Count - 1];
            disABBA = disAB;

            if (disAB[0] > disBA[0]) dmax = disBA[0]; disABBA = disBA;
            if (disAB[disAB.Count - 1] > disBA[disBA.Count - 1]) dmin = disBA[disBA.Count - 1];

            MaxandMin = new double[2] { dmax, dmin };

            return disABBA;
        }


        public void ComputeT(double accuracy, bool max, double angle, int alg)
        {


            Curve curvatureC = new PolylineCurve();
            //Declare our list of samples.
            List<Point3d> dir0 = SampleCurvature(accuracy, max, 0, alg);
            List<Point3d> dir1 = SampleCurvature(accuracy, max, Math.PI, alg);

            List<Point3d> outDir = new List<Point3d>();

            //test if the curvature line is CLOSED
            if (dir0[0] == dir0[dir0.Count - 1]) outDir = dir0;

            //Remove the first point in dir1 as it's a duplicate
            else
            {
                dir1.RemoveAt(0);
                dir1.Reverse();
                dir1.AddRange(dir0);

                outDir = dir1;
            }



            curvatureC = new PolylineCurve(outDir);


            curvatureC.Domain = new Interval(0.0, 1.0);
            Curve[] curvatureCPull=curvatureC.PullToBrepFace(((this.AgentSystem as RailAgentSystem).SystemSurface.ToBrep()).Faces[0],0.001);
            if (curvatureCPull.Length > 0) 
            { 
                var CCIntEvents = Rhino.Geometry.Intersect.Intersection.CurveCurve(this.GuideCurve, curvatureCPull[0], 5,5);

                if (CCIntEvents != null)
                {
                    for (int j = 0; j < CCIntEvents.Count; j++)
                    {
                        if (Math.Abs(this.t[0] - CCIntEvents[j].ParameterA) > 0.001) { this.t[1] = CCIntEvents[j].ParameterA;  }
                    }

                }
            }
        }


        public void FindNeighbors()
        {
            this.Neighbors = new RailAgent[2] { null, null };

            double thisT = this.t[0];
            int nextT=-1 ;
            int beforeT=-1;
            List<double> tSList = new List<double>();
            tSList.Add(thisT);
            foreach (RailAgent agentS in this.SameGuideCurce) { double[] dt = agentS.t; tSList.AddRange(dt); }

            tSList.Sort((x,y) => x.CompareTo(y));

            if (tSList.Contains(thisT))
            {
                int thisIndex=tSList.IndexOf(thisT);
                if(thisIndex == 0)    {  nextT = thisIndex + 1; }
                if(thisIndex==tSList.Count-1){ beforeT = thisIndex - 1; }
                if(thisIndex !=0 && thisIndex !=tSList.Count-1) { nextT = thisIndex + 1; beforeT = thisIndex - 1; }
            }

            foreach (RailAgent railA in SameGuideCurce)
            {
                List<double> tailAT = new List<double>();
                tailAT.AddRange(railA.t);

                if (nextT !=-1 && tailAT.Contains(tSList[nextT])) {this.Neighbors[1] = railA; }
                if (beforeT !=-1 && tailAT.Contains(tSList[beforeT])) { this.Neighbors[0] = railA; }
            }

            if (thisT <=0.001) this.Neighbors[0] = null;
            if (thisT >= 0.999) this.Neighbors[1] = null;
            
        }

        public List<Point3d> SampleCurvature(double accuracy, bool max, double angle, int alg)
        {
            Surface srf = (this.AgentSystem as RailAgentSystem).SystemSurface;


            Point3d p = new Point3d(this.UV.X, this.UV.Y, 0);
            Interval U = srf.Domain(0);
            Interval V = srf.Domain(1);

            List<Point3d> samples = new List<Point3d>();
            do
            {
                //Add the current point.
                samples.Add(srf.PointAt(p.X, p.Y));

                //####################################
                //##### Euler/Modified Euler/RK4 #####
                //####################################
                Vector3d dir = new Vector3d();

                switch (alg)
                {
                    case 1:
                        dir = Euler(srf, p, max, angle, accuracy, samples);
                        break;
                    case 2:
                        dir = ModEuler(srf, p, max, angle, accuracy, samples);
                        break;
                    case 3:
                        dir = RK4(srf, p, max, angle, accuracy, samples);
                        break;

                }

                if (ReferenceEquals(dir, null))
                {
                    break;
                }

                double s = 0;
                double t = 0;
                Point3d pt = samples[samples.Count - 1] + dir;
                if (!srf.ClosestPoint(pt, out s, out t))
                {
                    break;
                }

                //##################
                //##### Checks #####
                //##################
                //Abort if we've added more than 10,000 samples.
                if (samples.Count > 9999)
                {
                    break;
                }

                //Abort if we've wandered beyond the surface edge.
                if (!U.IncludesParameter(s, true) || !V.IncludesParameter(t, true))
                {
                    break;
                }
                /*if (!V.IncludesParameter(t, true))
                {
                  break;
                }*/

                //Abort if the new point is basically the same as the old point.
                if ((Math.Abs(p.X - s) < 1e-12) && (Math.Abs(p.Y - t) < 1e-12))
                {
                    break;
                }

                //Abort if the new point is basically the same as the start point.
                if (samples.Count > 1)
                {
                    Point3d q = srf.PointAt(this.UV.X, this.UV.Y);
                    if (pt.DistanceTo(q) < accuracy)
                    {
                        samples.Add(q);
                        break;
                    }
                }

                p.X = s;
                p.Y = t;
            } while (true);

            return samples;
        }

        private Vector3d GetDir(Surface srf, Point3d p, bool max, double angle, double h, Vector3d PrevDir)
        {
            //Get the curvature at the current point.
            SurfaceCurvature crv = srf.CurvatureAt(p.X, p.Y);

            //Get the maximum principal direction.
            Vector3d dir = new Vector3d();
            if (crv.Kappa(0) > crv.Kappa(1))
            {
                if (max)
                {
                    dir = crv.Direction(0);
                }
                else
                {
                    dir = crv.Direction(1);
                }
            }
            else
            {
                if (max)
                {
                    dir = crv.Direction(1);
                }
                else
                {
                    dir = crv.Direction(0);
                }
            }

            dir.Rotate(angle, crv.Normal);

            if (!dir.IsValid)
            {
                return default(Vector3d);
            }
            if (!dir.Unitize())
            {
                return default(Vector3d);
            }

            //Scale the direction vector to match our accuracy.
            dir *= h;

            //Flip the direction 180 degrees if it seems to be going backwards
            if (!ReferenceEquals(PrevDir, null))
            {
                if (dir.IsParallelTo(PrevDir, 0.5 * Math.PI) < 0)
                {
                    dir.Reverse();
                }
            }

            return dir;
        }

        private Vector3d Euler(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
        {
            int N = samples.Count;
            Vector3d PrevDir = new Vector3d();
            if (N > 1)
            {
                PrevDir = samples[N - 1] - samples[N - 2];
            }

            Vector3d dir = GetDir(srf, p, max, angle, h, PrevDir);
            if (ReferenceEquals(dir, null))
            {
                return default(Vector3d);
            }

            return dir;
        }

        private Vector3d ModEuler(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
        {
            int N = samples.Count;
            Vector3d PrevDir = new Vector3d();
            if (N > 1)
            {
                PrevDir = samples[N - 1] - samples[N - 2];
            }

            Vector3d dir1 = GetDir(srf, p, max, angle, h, PrevDir);
            if (ReferenceEquals(dir1, null))
            {
                return default(Vector3d);
            }

            //Move the last point in the list along the curvature direction.
            Point3d pt = samples[samples.Count - 1] + dir1;

            double s = 0;
            double t = 0;
            if (!srf.ClosestPoint(pt, out s, out t))
            {
                return default(Vector3d);
            }
            pt.X = s;
            pt.Y = t;

            Vector3d dir2 = GetDir(srf, pt, max, angle, h, dir1);
            if (ReferenceEquals(dir2, null))
            {
                return default(Vector3d);
            }

            Vector3d dir = (dir1 + dir2) * 0.5;
            return dir;
        }

        private Vector3d RK4(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
        {
            int N = samples.Count;
            Vector3d PrevDir = new Vector3d();
            if (N > 1)
            {
                PrevDir = samples[N - 1] - samples[N - 2];
            }

            Vector3d K1 = GetDir(srf, p, max, angle, h, PrevDir);
            if (ReferenceEquals(K1, null))
            {
                return default(Vector3d);
            }

            //Move the last point in the list along the curvature direction.
            Point3d pt1 = samples[samples.Count - 1] + K1 * 0.5;

            double s = 0;
            double t = 0;
            if (!srf.ClosestPoint(pt1, out s, out t))
            {
                return default(Vector3d);
            }
            pt1.X = s;
            pt1.Y = t;

            Vector3d K2 = GetDir(srf, pt1, max, angle, h, K1);
            if (ReferenceEquals(K2, null))
            {
                return default(Vector3d);
            }

            Point3d pt2 = samples[samples.Count - 1] + K2 * 0.5;

            if (!srf.ClosestPoint(pt2, out s, out t))
            {
                return default(Vector3d);
            }
            pt2.X = s;
            pt2.Y = t;

            Vector3d K3 = GetDir(srf, pt2, max, angle, h, K1);
            if (ReferenceEquals(K3, null))
            {
                return default(Vector3d);
            }

            Point3d pt3 = samples[samples.Count - 1] + K3;

            if (!srf.ClosestPoint(pt3, out s, out t))
            {
                return default(Vector3d);
            }
            pt3.X = s;
            pt3.Y = t;

            Vector3d K4 = GetDir(srf, pt3, max, angle, h, K1);
            if (ReferenceEquals(K4, null))
            {
                return default(Vector3d);
            }

            Vector3d dir = (double)1 / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
            return dir;
        }

    }




}

