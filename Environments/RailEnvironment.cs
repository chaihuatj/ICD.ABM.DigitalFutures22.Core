using ABxM.Core.Environments;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ICD.ABM.DigitalFutures22.Core.Environments
{
    public class RailEnvironment : SingleBrepEnvironment
    {
        public List<Brep> Rails;

        public RailEnvironment(Brep brep, List<Brep> rails)
        {
            BrepObject = brep;
            Rails = rails;
        }

        public RailEnvironment(Brep brep, List<Brep> rails, double spacing)
        {
            BrepObject = brep;
            Rails = rails;
            ComputeCurvatureField(spacing);
        }
    }
}
