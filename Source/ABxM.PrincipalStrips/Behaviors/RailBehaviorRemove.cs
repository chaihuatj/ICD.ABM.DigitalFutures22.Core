using ABxM.Core.Agent;
using ABxM.Core.Behavior;
using ABxM.PrinciaplStrips.Core.Agent;
using ABxM.PrinciaplStrips.Core.AgentSystem;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ABxM.PrincipalStrips.Core.Behaviors
{
    public class RailBehaviorRemove : BehaviorBase
    {
        public bool CurvatureDir;
        public double ThresholdMin;
        public double CurvatureAccuracy = 10;
        public int alg = 3;

        public RailBehaviorRemove(bool _Dir, double _ThresholdMin)
        {
            CurvatureDir = _Dir;
            Weight = 1.0;
            ThresholdMin = _ThresholdMin;
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

                    if (curveDis[1] < ThresholdMin * 0.1 && neighborAgent.isFixed != true)
                    {
                        mySystem.RemoveAgent(neighborAgent);
                    }

                }
            }
        }
    }

}
