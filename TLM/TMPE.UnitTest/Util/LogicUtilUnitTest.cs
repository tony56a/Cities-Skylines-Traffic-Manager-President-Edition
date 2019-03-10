using System;
using CSUtil.Commons;
using NUnit.Framework;

namespace TMUnitTest.Util {
	[TestFixture]
	public class LogicUtilUnitTest {
		[Test]
		public void TestCheckFlags1() {
			Assert.IsTrue(LogicUtil.CheckFlags((uint)(NetSegment.Flags.Created | NetSegment.Flags.Deleted), (uint)NetSegment.Flags.Created));
		}

		[Test]
		public void TestCheckFlags2() {
			Assert.IsFalse(LogicUtil.CheckFlags((uint)(NetSegment.Flags.Created | NetSegment.Flags.Deleted), (uint)NetSegment.Flags.Collapsed));
		}

		[Test]
		public void TestCheckFlags3() {
			Assert.IsTrue(LogicUtil.CheckFlags((uint)(NetSegment.Flags.Created | NetSegment.Flags.Collapsed), (uint)NetSegment.Flags.Created, (uint)NetSegment.Flags.Created));
		}

		[Test]
		public void TestCheckFlags4() {
			Assert.IsTrue(LogicUtil.CheckFlags((uint)(NetSegment.Flags.Created | NetSegment.Flags.Collapsed), (uint)(NetSegment.Flags.Created | NetSegment.Flags.Deleted), (uint)NetSegment.Flags.Created));
		}

		[Test]
		public void TestCheckFlags5() {
			Assert.IsFalse(LogicUtil.CheckFlags((uint)(NetSegment.Flags.Created | NetSegment.Flags.Deleted | NetSegment.Flags.Collapsed), (uint)(NetSegment.Flags.Created | NetSegment.Flags.Deleted), (uint)NetSegment.Flags.Created));
		}
	}
}
