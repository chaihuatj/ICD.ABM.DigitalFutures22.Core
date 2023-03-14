using ABxM.Core.Agent;
using ABxM.Core.Behavior;
using ABxM.PrincipalStrips.Core.Agent;
using ABxM.PrincipalStrips.Core.AgentSystem;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ABxM.PrincipalStrips.Core.Behaviors
{
    public class RailBehaviorAdd : BehaviorBase
    {
        public bool CurvatureDir;
        public double ThresholdMax;
        public double CurvatureAccuracy = 10;
        public int alg = 3;

        public RailBehaviorAdd(bool _Dir, double _ThresholdMax)
        {
            CurvatureDir = _Dir;
            Weight = 1.0;
            ThresholdMax = _ThresholdMax;
        }

        public override void Execute(AgentBase agent)
        {
            RailAgent myAgent = (RailAgent)agent;
            RailAgentSystem mySystem = (RailAgentSystem)myAgent.AgentSystem;

            //myAgent.FindGuideCurve();
            myAgent.FindSameGuideCurve();
            //myAgent.ComputeT(CurvatureAccuracy, CurvatureDir, 0, 3);
            myAgent.FindNeighbors();

            Curve Cc = new PolylineCurve();


            //Declare our list of samples.
            List<Point3d> dir0 = myAgent.SampleCurvature(CurvatureAccuracy, CurvatureDir, 0, alg);
            List<Point3d> dir1 = myAgent.SampleCurvature(CurvatureAccuracy, CurvatureDir, Math.PI, alg);

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



            Cc = new PolylineCurve(outDir);



            Cc.Domain = new Interval(0.0, 1.0);
            foreach (RailAgent neighborAgent in myAgent.Neighbors)
            {
                if (neighborAgent != null)
                {
                    Curve CNeighbor = new PolylineCurve();


                    //Declare our list of samples.
                    List<Point3d> dir2 = neighborAgent.SampleCurvature(CurvatureAccuracy, CurvatureDir, 0, alg);
                    List<Point3d> dir3 = neighborAgent.SampleCurvature(CurvatureAccuracy, CurvatureDir, Math.PI, alg);


                    List<Point3d> outDir1 = new List<Point3d>();

                    //test if the curvature line is CLOSED
                    if (dir2[0] == dir2[dir2.Count - 1]) outDir1 = dir2;

                    //Remove the first point in dir1 as it's a duplicate
                    else
                    {
                        dir3.RemoveAt(0);
                        dir3.Reverse();
                        dir3.AddRange(dir2);

                        outDir1 = dir3;
                    }




                    CNeighbor = new PolylineCurve(outDir1);



                    CNeighbor.Domain = new Interval(0.0, 1.0);
                    double[] curveDis = new double[2];
                    myAgent.ComputeCurveDistance(Cc, CNeighbor, out curveDis);


                    if (curveDis[0] > ThresholdMax * 2)
                    {
                        double t1 = myAgent.t[0];
                        double t2 = neighborAgent.t[0];
                        if (neighborAgent.t.Length > 1)
                        {
                            if (Math.Abs(neighborAgent.t[0] - myAgent.t[0]) > Math.Abs(neighborAgent.t[1] - myAgent.t[0]))
                            { t2 = neighborAgent.t[1]; }
                        }

                        double tMid = 0.5 * (t1 + t2);

                        Point3d newPoint = myAgent.GuideCurve.PointAt(tMid);
                        double u1, v1;
                        mySystem.SystemSurface.ClosestPoint(newPoint, out u1, out v1);

                        mySystem.AddAgent(new RailAgent(new Point2d(u1, v1), myAgent.Behaviors));
                    }

                }
            }
        }
    }

}
