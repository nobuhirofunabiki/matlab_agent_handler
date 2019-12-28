addpath(genpath('../../Agent'));
import matlab.unittest.TestSuite
suiteClass = TestSuite.fromClass(?TestAgentHandlerEstimate);
result = run(suiteClass);