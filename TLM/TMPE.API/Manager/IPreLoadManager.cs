using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TrafficManager.Manager
{

		public interface IPreLoadManager
		{
				/// <summary>
				/// Load any data that is required to be loaded in the LoadingExtenion
				/// </summary>
				void PreLoadData();
		}
}
