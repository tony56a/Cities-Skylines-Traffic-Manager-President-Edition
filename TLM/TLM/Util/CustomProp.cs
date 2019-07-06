using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace TrafficManager.Util {
		public class CustomProp {

				public CustomProp(Material material, Mesh mesh) {
						this.Material = material;
						this.Mesh = mesh;
				}
				
				public Material Material { get; }
				public Mesh Mesh { get; }
		}
}
