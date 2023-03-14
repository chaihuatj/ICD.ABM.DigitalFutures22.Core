using ABxM.Core.Agent;
using ABxM.Core.AgentSystem;
using ABxM.PrincipalStrips.Core.Agent;
using Rhino.Geometry;
using System.Collections.Generic;

namespace ABxM.PrincipalStrips.Core.AgentSystem
{
    public class RailAgentSystem : AgentSystemBase
    {
        public Surface SystemSurface = null;
        public List<Curve> SystemCurves = new List<Curve>();
        public List<double> moveDistances = new List<double>();
        public double stopThreshold = 1 / 1000;

        public RailAgentSystem(List<RailAgent> agents, double iStopThreshold)
        {
            Agents = new List<AgentBase>();
            for (int i = 0; i < agents.Count; ++i)
            {
                agents[i].Id = i;
                agents[i].AgentSystem = this;
                Agents.Add(agents[i]);
            }
            stopThreshold = iStopThreshold;
        }


        public override void Reset()
        {
            base.Reset();
            moveDistances = new List<double>();

        }
        public override void PreExecute()
        {
            base.PreExecute();
            moveDistances = new List<double>();

        }

        public override void Execute()
        {
            if (this.IsFinished())
                return;
            base.Execute();
        }

        public override void PostExecute()
        {
            base.PostExecute();
        }


        public override bool IsFinished()
        {
            bool isF = false;
            double sumdis = new double();
            if (moveDistances.Count > 0)
            {
                foreach (double d in moveDistances)
                    sumdis += d;
                if (sumdis < stopThreshold) isF = true;
            }
            else isF = false;
            return isF;
        }

    }
}
