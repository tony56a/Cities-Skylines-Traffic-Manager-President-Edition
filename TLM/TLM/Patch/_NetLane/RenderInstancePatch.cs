using ColossalFramework.Math;
using CSUtil.Commons;
using Harmony;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Emit;
using TrafficManager.Manager.Impl;
using UnityEngine;

namespace TrafficManager.Patch._NetLane
{
		[HarmonyPatch(typeof(NetLane), "RenderInstance")]
		public static class RenderInstancePatch
		{

				/// <summary>
				/// Overrides the PropInfo retrieval method for Netlane.RenderInstancePatch to allow the LanePropManager to override the prop selection
				/// </summary>
				/// <param name="instructions">The Bytecode for the original Netlane.RenderInstance method</param>
				[HarmonyTranspiler]
				public static IEnumerable<CodeInstruction> ReplacePropLoad(IEnumerable<CodeInstruction> instructions) {
						// Create a copy of the method as a list to simplify iteration and insertion
						var codes = new List<CodeInstruction>(instructions);

						for (int i = 0; i < codes.Count; i++)
						{
								if(codes[i].ToString() == "ldfld PropInfo m_finalProp")
								{

										// Once we've found the entrypoint to replace, override the original call (PropInfo propInfo = prop.m_finalProp)
										// with the equivleant of (PropInfo propInfo = LanePropManager.Instance.ReplaceProp())
										Log._Debug("Found bytecode entry point for replacement");
										// Copy the label, as the Runtime requires it to correctly run the updated method
										var originalLabel = codes[i - 1].labels;
										var getManagerInstanceInstruction = new CodeInstruction(OpCodes.Ldsfld, AccessTools.Field(typeof(LanePropManager), nameof(LanePropManager.Instance)));
										getManagerInstanceInstruction.labels = originalLabel;
										codes[i - 1] = getManagerInstanceInstruction;
										codes[i] = new CodeInstruction(OpCodes.Ldloc_S, 12);
										codes.Insert(i + 1, new CodeInstruction(OpCodes.Ldarg_2));
										codes.Insert(i + 2, new CodeInstruction(OpCodes.Ldarg_3));
										codes.Insert(i + 3, new CodeInstruction(OpCodes.Callvirt, AccessTools.Method(typeof(LanePropManager), "ReplaceProp")));
								}
						}

						return codes.AsEnumerable();
				}
		}
}
