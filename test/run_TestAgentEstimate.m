addpath(genpath('../../Agent'));
import matlab.unittest.TestSuite
suiteClass = TestSuite.fromClass(?TestAgentEstimate);
result = run(suiteClass);