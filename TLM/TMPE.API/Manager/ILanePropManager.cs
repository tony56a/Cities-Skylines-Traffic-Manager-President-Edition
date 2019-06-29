using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace TrafficManager.Manager {
		public interface ILanePropManager
		{
				/// <summary>
				/// Replace the Lane Prop, based on the original NetLane prop, as well asthe current segment/lane configurations
				/// </summary>
				/// <param name="originalNetLaneProp">The original Netlane prop to be rendered</param>
				/// <param name="segmentId">The segment the prop is on</param>
				/// <param name="laneId">The lane the prop is on</param>
				/// <returns>A new PropInfo to render, or the original if no replacement is available</returns>
				PropInfo ReplaceProp(NetLaneProps.Prop originalNetLaneProp, ushort segmentId, uint laneId);
		}
}
