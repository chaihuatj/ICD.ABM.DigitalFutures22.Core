using ABxM.Core.Agent;
using ABxM.Core.AgentSystem;
using ICD.ABM.DigitalFutures22.Core.Agent;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ICD.ABM.DigitalFutures22.Core.AgentSystem
{
    public class UmbilicalAgentSystem : AgentSystemBase
    {
        public Surface SystemSurface = null;

        public UmbilicalAgentSystem(List<UmbilicalAgent> agents)
        {
            Agents = new List<AgentBase>();
            for (int i = 0; i < agents.Count; ++i)
            {
                agents[i].Id = i;
                agents[i].AgentSystem = this;
                Agents.Add(agents[i]);
            }
        }

        public override void Reset()
        {
            base.Reset();
        }

        public override void PreExecute()
        {
            base.PreExecute();
        }

        public override void Execute()
        {
            base.Execute();
        }

        public override void PostExecute()
        {
            base.PostExecute();
        }

        public override List<object> GetDisplayGeometries()
        {
            List<object> displayGeometry = new List<object>();

            foreach (UmbilicalAgent agent in Agents)
                displayGeometry.AddRange(agent.GetDisplayGeometries());

            return displayGeometry;
        }

    }
}
