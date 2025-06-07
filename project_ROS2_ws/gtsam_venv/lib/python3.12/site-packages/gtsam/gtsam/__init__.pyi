"""
pybind11 wrapper of hybrid
"""
from __future__ import annotations
import numpy
import typing
from . import gtsfm
from . import imuBias
from . import lago
from . import noiseModel
from . import so3
from . import symbol_shorthand
from . import utilities
__all__ = ['AHRSFactor', 'AcceleratingScenario', 'BarometricFactor', 'BatchFixedLagSmoother', 'BearingFactor2D', 'BearingFactor3D', 'BearingFactorPose2', 'BearingRange2D', 'BearingRange3D', 'BearingRangeFactor2D', 'BearingRangeFactor3D', 'BearingRangeFactorPose2', 'BearingRangeFactorPose3', 'BearingRangePose2', 'BearingRangePose3', 'BetweenFactorConstantBias', 'BetweenFactorDouble', 'BetweenFactorPoint2', 'BetweenFactorPoint3', 'BetweenFactorPose2', 'BetweenFactorPose3', 'BetweenFactorRot2', 'BetweenFactorRot3', 'BetweenFactorSO3', 'BetweenFactorSO4', 'BetweenFactorSimilarity2', 'BetweenFactorSimilarity3', 'BetweenFactorVector', 'BinaryMeasurementPoint3', 'BinaryMeasurementRot3', 'BinaryMeasurementUnit3', 'BlockJacobiPreconditionerParameters', 'Cal3', 'Cal3Bundler', 'Cal3DS2', 'Cal3DS2_Base', 'Cal3Fisheye', 'Cal3Unified', 'Cal3_S2', 'Cal3_S2Stereo', 'Cal3f', 'CalibratedCamera', 'CameraSetCal3Bundler', 'CameraSetCal3DS2', 'CameraSetCal3Fisheye', 'CameraSetCal3Unified', 'CameraSetCal3_S2', 'CameraSetPinholePoseCal3_S2', 'Chebyshev1Basis', 'Chebyshev2', 'Chebyshev2Basis', 'CombinedImuFactor', 'ComponentDerivativeFactorChebyshev1Basis', 'ComponentDerivativeFactorChebyshev2', 'ComponentDerivativeFactorChebyshev2Basis', 'ComponentDerivativeFactorFourierBasis', 'ConjugateGradientParameters', 'ConstantTwistScenario', 'ConstantVelocityFactor', 'ConvertNoiseModel', 'CustomFactor', 'DSFMapIndexPair', 'DecisionTreeFactor', 'DegeneracyMode', 'DerivativeFactorChebyshev1Basis', 'DerivativeFactorChebyshev2', 'DerivativeFactorChebyshev2Basis', 'DerivativeFactorFourierBasis', 'DiscreteBayesNet', 'DiscreteBayesTree', 'DiscreteBayesTreeClique', 'DiscreteCluster', 'DiscreteConditional', 'DiscreteDistribution', 'DiscreteEliminationTree', 'DiscreteFactor', 'DiscreteFactorGraph', 'DiscreteJunctionTree', 'DiscreteKeys', 'DiscreteLookupDAG', 'DiscreteLookupTable', 'DiscreteMarginals', 'DiscreteSearch', 'DiscreteSearchSolution', 'DiscreteValues', 'DoglegOptimizer', 'DoglegParams', 'DotWriter', 'DummyPreconditionerParameters', 'EdgeKey', 'EliminateDiscrete', 'EliminateForMPE', 'EliminateQR', 'EpipolarTransfer', 'EssentialMatrix', 'EssentialMatrixConstraint', 'EssentialMatrixFactor', 'EssentialMatrixFactor2', 'EssentialMatrixFactor3', 'EssentialMatrixFactor4Cal3Bundler', 'EssentialMatrixFactor4Cal3DS2', 'EssentialMatrixFactor4Cal3Fisheye', 'EssentialMatrixFactor4Cal3Unified', 'EssentialMatrixFactor4Cal3_S2', 'EssentialMatrixFactor4Cal3f', 'EssentialMatrixFactor5Cal3Bundler', 'EssentialMatrixFactor5Cal3DS2', 'EssentialMatrixFactor5Cal3Fisheye', 'EssentialMatrixFactor5Cal3Unified', 'EssentialMatrixFactor5Cal3_S2', 'EssentialMatrixFactor5Cal3f', 'EssentialTransferFactorCal3Bundler', 'EssentialTransferFactorCal3_S2', 'EssentialTransferFactorCal3f', 'EssentialTransferFactorKCal3Bundler', 'EssentialTransferFactorKCal3_S2', 'EssentialTransferFactorKCal3f', 'EvaluationFactorChebyshev1Basis', 'EvaluationFactorChebyshev2', 'EvaluationFactorChebyshev2Basis', 'EvaluationFactorFourierBasis', 'Event', 'ExtendedKalmanFilterConstantBias', 'ExtendedKalmanFilterNavState', 'ExtendedKalmanFilterPoint2', 'ExtendedKalmanFilterPoint3', 'ExtendedKalmanFilterPose2', 'ExtendedKalmanFilterPose3', 'ExtendedKalmanFilterRot2', 'ExtendedKalmanFilterRot3', 'ExtendedKalmanFilterSimilarity2', 'ExtendedKalmanFilterSimilarity3', 'Factor', 'FindKarcherMeanPoint2', 'FindKarcherMeanPoint3', 'FindKarcherMeanPose2', 'FindKarcherMeanPose3', 'FindKarcherMeanRot2', 'FindKarcherMeanRot3', 'FindKarcherMeanSO3', 'FindKarcherMeanSO4', 'FitBasisChebyshev1Basis', 'FitBasisChebyshev2', 'FitBasisChebyshev2Basis', 'FitBasisFourierBasis', 'FixedLagSmoother', 'FixedLagSmootherResult', 'FourierBasis', 'FrobeniusBetweenFactorPose2', 'FrobeniusBetweenFactorPose3', 'FrobeniusBetweenFactorRot2', 'FrobeniusBetweenFactorRot3', 'FrobeniusBetweenFactorSO3', 'FrobeniusBetweenFactorSO4', 'FrobeniusFactorPose2', 'FrobeniusFactorPose3', 'FrobeniusFactorRot2', 'FrobeniusFactorRot3', 'FrobeniusFactorSO3', 'FrobeniusFactorSO4', 'FrobeniusPriorPose2', 'FrobeniusPriorPose3', 'FrobeniusPriorRot2', 'FrobeniusPriorRot3', 'FrobeniusPriorSO3', 'FrobeniusPriorSO4', 'FundamentalMatrix', 'GPSFactor', 'GPSFactor2', 'GPSFactor2Arm', 'GPSFactor2ArmCalib', 'GPSFactorArm', 'GPSFactorArmCalib', 'GaussNewtonOptimizer', 'GaussNewtonParams', 'GaussianBayesNet', 'GaussianBayesTree', 'GaussianBayesTreeClique', 'GaussianConditional', 'GaussianDensity', 'GaussianEliminationTree', 'GaussianFactor', 'GaussianFactorGraph', 'GaussianISAM', 'GeneralSFMFactor2Cal3Bundler', 'GeneralSFMFactor2Cal3DS2', 'GeneralSFMFactor2Cal3Fisheye', 'GeneralSFMFactor2Cal3Unified', 'GeneralSFMFactor2Cal3_S2', 'GeneralSFMFactor2Cal3f', 'GeneralSFMFactorCal3Bundler', 'GeneralSFMFactorCal3DS2', 'GeneralSFMFactorCal3Fisheye', 'GeneralSFMFactorCal3Unified', 'GeneralSFMFactorCal3_S2', 'GeneralSFMFactorPoseCal3Bundler', 'GeneralSFMFactorPoseCal3DS2', 'GeneralSFMFactorPoseCal3Fisheye', 'GeneralSFMFactorPoseCal3Unified', 'GeneralSFMFactorPoseCal3_S2', 'GenericProjectionFactorCal3DS2', 'GenericProjectionFactorCal3Fisheye', 'GenericProjectionFactorCal3Unified', 'GenericProjectionFactorCal3_S2', 'GenericStereoFactor3D', 'GenericValueCal3Bundler', 'GenericValueCal3DS2', 'GenericValueCal3Fisheye', 'GenericValueCal3Unified', 'GenericValueCal3_S2', 'GenericValueCalibratedCamera', 'GenericValueConstantBias', 'GenericValueEssentialMatrix', 'GenericValueMatrix', 'GenericValuePoint2', 'GenericValuePoint3', 'GenericValuePose2', 'GenericValuePose3', 'GenericValueRot2', 'GenericValueRot3', 'GenericValueStereoPoint2', 'GenericValueVector', 'GncGaussNewtonOptimizer', 'GncGaussNewtonParams', 'GncLMOptimizer', 'GncLMParams', 'GncLossType', 'GraphvizFormatting', 'HessianFactor', 'HybridBayesNet', 'HybridBayesTree', 'HybridBayesTreeClique', 'HybridConditional', 'HybridFactor', 'HybridGaussianConditional', 'HybridGaussianFactor', 'HybridGaussianFactorGraph', 'HybridNonlinearFactor', 'HybridNonlinearFactorGraph', 'HybridSmoother', 'HybridValues', 'ISAM2', 'ISAM2Clique', 'ISAM2DoglegParams', 'ISAM2GaussNewtonParams', 'ISAM2Params', 'ISAM2Result', 'ISAM2ThresholdMap', 'ImuFactor', 'ImuFactor2', 'IncrementalFixedLagSmoother', 'IndexPair', 'IndexPairSetAsArray', 'InitializePose3', 'IterativeOptimizationParameters', 'JacobianFactor', 'JacobianVector', 'JointMarginal', 'KalmanFilter', 'KarcherMeanFactorPoint2', 'KarcherMeanFactorPoint3', 'KarcherMeanFactorPose2', 'KarcherMeanFactorPose3', 'KarcherMeanFactorRot2', 'KarcherMeanFactorRot3', 'KarcherMeanFactorSO3', 'KarcherMeanFactorSO4', 'KernelFunctionType', 'KeyGroupMap', 'KeyList', 'KeySet', 'LabeledSymbol', 'LevenbergMarquardtOptimizer', 'LevenbergMarquardtParams', 'LinearContainerFactor', 'LinearizationMode', 'MFAS', 'MT19937', 'MagFactor', 'MagFactor1', 'MagPoseFactorPose2', 'MagPoseFactorPose3', 'ManifoldEvaluationFactorChebyshev1BasisPose2', 'ManifoldEvaluationFactorChebyshev1BasisPose3', 'ManifoldEvaluationFactorChebyshev1BasisRot2', 'ManifoldEvaluationFactorChebyshev1BasisRot3', 'ManifoldEvaluationFactorChebyshev2BasisPose2', 'ManifoldEvaluationFactorChebyshev2BasisPose3', 'ManifoldEvaluationFactorChebyshev2BasisRot2', 'ManifoldEvaluationFactorChebyshev2BasisRot3', 'ManifoldEvaluationFactorChebyshev2Pose2', 'ManifoldEvaluationFactorChebyshev2Pose3', 'ManifoldEvaluationFactorChebyshev2Rot2', 'ManifoldEvaluationFactorChebyshev2Rot3', 'ManifoldEvaluationFactorFourierBasisPose2', 'ManifoldEvaluationFactorFourierBasisPose3', 'ManifoldEvaluationFactorFourierBasisRot2', 'ManifoldEvaluationFactorFourierBasisRot3', 'Marginals', 'NavState', 'NoiseFormat', 'NoiseModelFactor', 'NonlinearEquality2Cal3_S2', 'NonlinearEquality2CalibratedCamera', 'NonlinearEquality2ConstantBias', 'NonlinearEquality2PinholeCameraCal3Bundler', 'NonlinearEquality2PinholeCameraCal3Fisheye', 'NonlinearEquality2PinholeCameraCal3Unified', 'NonlinearEquality2PinholeCameraCal3_S2', 'NonlinearEquality2Point2', 'NonlinearEquality2Point3', 'NonlinearEquality2Pose2', 'NonlinearEquality2Pose3', 'NonlinearEquality2Rot2', 'NonlinearEquality2Rot3', 'NonlinearEquality2SO3', 'NonlinearEquality2SO4', 'NonlinearEquality2SOn', 'NonlinearEquality2Similarity2', 'NonlinearEquality2Similarity3', 'NonlinearEquality2StereoPoint2', 'NonlinearEqualityCal3_S2', 'NonlinearEqualityCalibratedCamera', 'NonlinearEqualityConstantBias', 'NonlinearEqualityPinholeCameraCal3Bundler', 'NonlinearEqualityPinholeCameraCal3Fisheye', 'NonlinearEqualityPinholeCameraCal3Unified', 'NonlinearEqualityPinholeCameraCal3_S2', 'NonlinearEqualityPoint2', 'NonlinearEqualityPoint3', 'NonlinearEqualityPose2', 'NonlinearEqualityPose3', 'NonlinearEqualityRot2', 'NonlinearEqualityRot3', 'NonlinearEqualitySO3', 'NonlinearEqualitySO4', 'NonlinearEqualitySOn', 'NonlinearEqualitySimilarity2', 'NonlinearEqualitySimilarity3', 'NonlinearEqualityStereoPoint2', 'NonlinearFactor', 'NonlinearFactorGraph', 'NonlinearISAM', 'NonlinearOptimizer', 'NonlinearOptimizerParams', 'Ordering', 'OrientedPlane3', 'OrientedPlane3DirectionPrior', 'OrientedPlane3Factor', 'PCGSolverParameters', 'PinholeCameraCal3Bundler', 'PinholeCameraCal3DS2', 'PinholeCameraCal3Fisheye', 'PinholeCameraCal3Unified', 'PinholeCameraCal3_S2', 'PinholeCameraCal3f', 'PinholePoseCal3Bundler', 'PinholePoseCal3DS2', 'PinholePoseCal3Fisheye', 'PinholePoseCal3Unified', 'PinholePoseCal3_S2', 'PlanarProjectionFactor1', 'PlanarProjectionFactor2', 'PlanarProjectionFactor3', 'Pose2', 'Pose3', 'Pose3AttitudeFactor', 'PoseRotationPrior2D', 'PoseRotationPrior3D', 'PoseTranslationPrior2D', 'PoseTranslationPrior3D', 'PreconditionerParameters', 'PreintegratedAhrsMeasurements', 'PreintegratedCombinedMeasurements', 'PreintegratedImuMeasurements', 'PreintegratedRotation', 'PreintegratedRotationParams', 'PreintegrationCombinedParams', 'PreintegrationParams', 'PrintDiscreteValues', 'PrintKeyList', 'PrintKeySet', 'PrintKeyVector', 'PriorFactorCal3Bundler', 'PriorFactorCal3DS2', 'PriorFactorCal3Fisheye', 'PriorFactorCal3Unified', 'PriorFactorCal3_S2', 'PriorFactorCalibratedCamera', 'PriorFactorConstantBias', 'PriorFactorDouble', 'PriorFactorNavState', 'PriorFactorPinholeCameraCal3Bundler', 'PriorFactorPinholeCameraCal3Fisheye', 'PriorFactorPinholeCameraCal3Unified', 'PriorFactorPinholeCameraCal3_S2', 'PriorFactorPoint2', 'PriorFactorPoint3', 'PriorFactorPose2', 'PriorFactorPose3', 'PriorFactorRot2', 'PriorFactorRot3', 'PriorFactorSO3', 'PriorFactorSO4', 'PriorFactorSOn', 'PriorFactorSimilarity2', 'PriorFactorSimilarity3', 'PriorFactorStereoPoint2', 'PriorFactorUnit3', 'PriorFactorVector', 'Quaternion', 'RangeFactor2', 'RangeFactor2D', 'RangeFactor3', 'RangeFactor3D', 'RangeFactorCalibratedCamera', 'RangeFactorCalibratedCameraPoint', 'RangeFactorPose2', 'RangeFactorPose3', 'RangeFactorSimpleCamera', 'RangeFactorSimpleCameraPoint', 'RangeFactorWithTransform2D', 'RangeFactorWithTransform3D', 'RangeFactorWithTransformPose2', 'RangeFactorWithTransformPose3', 'RedirectCout', 'ReferenceFrameFactorPoint3Pose3', 'Rot2', 'Rot3', 'Rot3AttitudeFactor', 'RotateDirectionsFactor', 'RotateFactor', 'SO3', 'SO4', 'SOn', 'Sampler', 'Scenario', 'ScenarioRunner', 'SfmData', 'SfmTrack', 'SfmTrack2d', 'ShonanAveraging2', 'ShonanAveraging3', 'ShonanAveragingParameters2', 'ShonanAveragingParameters3', 'ShonanFactor3', 'Similarity2', 'Similarity3', 'SimpleFundamentalMatrix', 'SmartFactorBasePinholeCameraCal3Bundler', 'SmartFactorBasePinholeCameraCal3DS2', 'SmartFactorBasePinholeCameraCal3Fisheye', 'SmartFactorBasePinholeCameraCal3Unified', 'SmartFactorBasePinholeCameraCal3_S2', 'SmartFactorBasePinholePoseCal3Bundler', 'SmartFactorBasePinholePoseCal3DS2', 'SmartFactorBasePinholePoseCal3Fisheye', 'SmartFactorBasePinholePoseCal3Unified', 'SmartFactorBasePinholePoseCal3_S2', 'SmartProjectionFactorPinholeCameraCal3Bundler', 'SmartProjectionFactorPinholeCameraCal3DS2', 'SmartProjectionFactorPinholeCameraCal3Fisheye', 'SmartProjectionFactorPinholeCameraCal3Unified', 'SmartProjectionFactorPinholeCameraCal3_S2', 'SmartProjectionFactorPinholePoseCal3Bundler', 'SmartProjectionFactorPinholePoseCal3DS2', 'SmartProjectionFactorPinholePoseCal3Fisheye', 'SmartProjectionFactorPinholePoseCal3Unified', 'SmartProjectionFactorPinholePoseCal3_S2', 'SmartProjectionParams', 'SmartProjectionPoseFactorCal3Bundler', 'SmartProjectionPoseFactorCal3DS2', 'SmartProjectionPoseFactorCal3Fisheye', 'SmartProjectionPoseFactorCal3Unified', 'SmartProjectionPoseFactorCal3_S2', 'SmartProjectionRigFactorPinholePoseCal3Bundler', 'SmartProjectionRigFactorPinholePoseCal3DS2', 'SmartProjectionRigFactorPinholePoseCal3Fisheye', 'SmartProjectionRigFactorPinholePoseCal3Unified', 'SmartProjectionRigFactorPinholePoseCal3_S2', 'StereoCamera', 'StereoPoint2', 'SubgraphSolver', 'SubgraphSolverParameters', 'Symbol', 'SymbolicBayesNet', 'SymbolicBayesTree', 'SymbolicBayesTreeClique', 'SymbolicCluster', 'SymbolicConditional', 'SymbolicEliminationTree', 'SymbolicFactor', 'SymbolicFactorGraph', 'SymbolicJunctionTree', 'TableDistribution', 'TableFactor', 'TransferFactorFundamentalMatrix', 'TransferFactorSimpleFundamentalMatrix', 'TranslationRecovery', 'TriangulationFactorCal3Bundler', 'TriangulationFactorCal3DS2', 'TriangulationFactorCal3Fisheye', 'TriangulationFactorCal3Unified', 'TriangulationFactorCal3_S2', 'TriangulationFactorPoseCal3Bundler', 'TriangulationFactorPoseCal3DS2', 'TriangulationFactorPoseCal3Fisheye', 'TriangulationFactorPoseCal3Unified', 'TriangulationFactorPoseCal3_S2', 'TriangulationParameters', 'TriangulationResult', 'Unit3', 'Value', 'Values', 'VariableIndex', 'VectorComponentFactorChebyshev1Basis', 'VectorComponentFactorChebyshev2', 'VectorComponentFactorChebyshev2Basis', 'VectorComponentFactorFourierBasis', 'VectorDerivativeFactorChebyshev1Basis', 'VectorDerivativeFactorChebyshev2', 'VectorDerivativeFactorChebyshev2Basis', 'VectorDerivativeFactorFourierBasis', 'VectorEvaluationFactorChebyshev1Basis', 'VectorEvaluationFactorChebyshev2', 'VectorEvaluationFactorChebyshev2Basis', 'VectorEvaluationFactorFourierBasis', 'VectorValues', 'cartesianProduct', 'checkConvergence', 'gtsfm', 'html', 'imuBias', 'initialCamerasAndPointsEstimate', 'initialCamerasEstimate', 'isDebugVersion', 'lago', 'linear_independent', 'load2D', 'load3D', 'markdown', 'mrsymbol', 'mrsymbolChr', 'mrsymbolIndex', 'mrsymbolLabel', 'noiseModel', 'parse2DFactors', 'parse3DFactors', 'readBal', 'readG2o', 'save2D', 'so3', 'symbol', 'symbolChr', 'symbolIndex', 'symbol_shorthand', 'triangulateNonlinear', 'triangulatePoint3', 'triangulateSafe', 'utilities', 'writeBAL', 'writeG2o']
class AHRSFactor(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, rot_i: int, rot_j: int, bias: int, preintegratedMeasurements: PreintegratedAhrsMeasurements) -> None:
        ...
    @typing.overload
    def __init__(self, rot_i: int, rot_j: int, bias: int, preintegratedMeasurements: PreintegratedAhrsMeasurements, omegaCoriolis: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, rot_i: int, rot_j: int, bias: int, preintegratedMeasurements: PreintegratedAhrsMeasurements, omegaCoriolis: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], body_P_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def evaluateError(self, rot_i: Rot3, rot_j: Rot3, bias: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def predict(self, rot_i: Rot3, bias: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], preintegratedMeasurements: PreintegratedAhrsMeasurements, omegaCoriolis: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        ...
    def preintegratedMeasurements(self) -> PreintegratedAhrsMeasurements:
        """
        Access the preintegrated measurements.
        """
    def serialize(self) -> str:
        ...
class AcceleratingScenario(Scenario):
    def __init__(self, nRb: Rot3, p0: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], v0: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], a_n: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], omega_b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
class BarometricFactor(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, baroKey: int, baroIn: float, model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def baroOut(self, meters: float) -> float:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, p: Pose3, b: float) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def heightOut(self, n: float) -> float:
        ...
    def measurementIn(self) -> float:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class BatchFixedLagSmoother(FixedLagSmoother):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, smootherLag: float) -> None:
        ...
    @typing.overload
    def __init__(self, smootherLag: float, parameters: LevenbergMarquardtParams) -> None:
        ...
    def __repr__(self, s: str = 'BatchFixedLagSmoother:\n') -> str:
        ...
    def calculateEstimateCal3DS2(self, key: int) -> Cal3DS2:
        ...
    def calculateEstimateCal3_S2(self, key: int) -> Cal3_S2:
        ...
    def calculateEstimateMatrix(self, key: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def calculateEstimatePoint2(self, key: int) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calculateEstimatePoint3(self, key: int) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calculateEstimatePose2(self, key: int) -> Pose2:
        ...
    def calculateEstimatePose3(self, key: int) -> Pose3:
        ...
    def calculateEstimateRot2(self, key: int) -> Rot2:
        ...
    def calculateEstimateRot3(self, key: int) -> Rot3:
        ...
    def calculateEstimateSimilarity2(self, key: int) -> Similarity2:
        ...
    def calculateEstimateSimilarity3(self, key: int) -> Similarity3:
        ...
    def calculateEstimateVector(self, key: int) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getFactors(self) -> NonlinearFactorGraph:
        """
        Access the current set of factors.
        """
    def params(self) -> LevenbergMarquardtParams:
        """
        read the current set of optimizer parameters
        """
    def print(self, s: str = 'BatchFixedLagSmoother:\n') -> None:
        ...
class BearingFactor2D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: Rot2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Rot2:
        ...
    def serialize(self) -> str:
        ...
class BearingFactor3D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: Unit3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Unit3:
        ...
    def serialize(self) -> str:
        ...
class BearingFactorPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: Rot2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Rot2:
        ...
    def serialize(self) -> str:
        ...
class BearingRange2D:
    @staticmethod
    def Measure(a1: Pose2, a2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> BearingRange2D:
        ...
    @staticmethod
    def MeasureBearing(a1: Pose2, a2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot2:
        ...
    @staticmethod
    def MeasureRange(a1: Pose2, a2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    def __init__(self, b: Rot2, r: float) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def bearing(self) -> Rot2:
        ...
    def print(self, str: str = '') -> None:
        ...
    def range(self) -> float:
        ...
class BearingRange3D:
    @staticmethod
    def Measure(a1: Pose3, a2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> BearingRange3D:
        ...
    @staticmethod
    def MeasureBearing(a1: Pose3, a2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Unit3:
        ...
    @staticmethod
    def MeasureRange(a1: Pose3, a2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    def __init__(self, b: Unit3, r: float) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def bearing(self) -> Unit3:
        ...
    def print(self, str: str = '') -> None:
        ...
    def range(self) -> float:
        ...
class BearingRangeFactor2D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, pointKey: int, measuredBearing: Rot2, measuredRange: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> BearingRange2D:
        ...
    def serialize(self) -> str:
        ...
class BearingRangeFactor3D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, pointKey: int, measuredBearing: Unit3, measuredRange: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> BearingRange3D:
        ...
    def serialize(self) -> str:
        ...
class BearingRangeFactorPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, pointKey: int, measuredBearing: Rot2, measuredRange: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> BearingRangePose2:
        ...
    def serialize(self) -> str:
        ...
class BearingRangeFactorPose3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, pointKey: int, measuredBearing: Unit3, measuredRange: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> BearingRangePose3:
        ...
    def serialize(self) -> str:
        ...
class BearingRangePose2:
    @staticmethod
    def Measure(a1: Pose2, a2: Pose2) -> BearingRangePose2:
        ...
    @staticmethod
    def MeasureBearing(a1: Pose2, a2: Pose2) -> Rot2:
        ...
    @staticmethod
    def MeasureRange(a1: Pose2, a2: Pose2) -> float:
        ...
    def __init__(self, b: Rot2, r: float) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def bearing(self) -> Rot2:
        ...
    def print(self, str: str = '') -> None:
        ...
    def range(self) -> float:
        ...
class BearingRangePose3:
    @staticmethod
    def Measure(a1: Pose3, a2: Pose3) -> BearingRangePose3:
        ...
    @staticmethod
    def MeasureBearing(a1: Pose3, a2: Pose3) -> Unit3:
        ...
    @staticmethod
    def MeasureRange(a1: Pose3, a2: Pose3) -> float:
        ...
    def __init__(self, b: Unit3, r: float) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def bearing(self) -> Unit3:
        ...
    def print(self, str: str = '') -> None:
        ...
    def range(self) -> float:
        ...
class BetweenFactorConstantBias(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: ..., noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> ...:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorDouble(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorPoint2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorPoint3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: Pose2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Pose2:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorPose3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: Pose3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Pose3:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorRot2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: Rot2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Rot2:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorRot3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: Rot3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Rot3:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorSO3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: SO3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> SO3:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorSO4(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: SO4, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> SO4:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorSimilarity2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: Similarity2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Similarity2:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorSimilarity3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: Similarity3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> Similarity3:
        ...
    def serialize(self) -> str:
        ...
class BetweenFactorVector(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, relativePose: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class BinaryMeasurementPoint3:
    def __init__(self, key1: int, key2: int, measured: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def key1(self) -> int:
        ...
    def key2(self) -> int:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def noiseModel(self) -> noiseModel.Base:
        ...
class BinaryMeasurementRot3:
    def __init__(self, key1: int, key2: int, measured: Rot3, model: noiseModel.Base) -> None:
        ...
    def key1(self) -> int:
        ...
    def key2(self) -> int:
        ...
    def measured(self) -> Rot3:
        ...
    def noiseModel(self) -> noiseModel.Base:
        ...
class BinaryMeasurementUnit3:
    def __init__(self, key1: int, key2: int, measured: Unit3, model: noiseModel.Base) -> None:
        ...
    def key1(self) -> int:
        ...
    def key2(self) -> int:
        ...
    def measured(self) -> Unit3:
        ...
    def noiseModel(self) -> noiseModel.Base:
        ...
class BlockJacobiPreconditionerParameters(PreconditionerParameters):
    def __init__(self) -> None:
        ...
class Cal3:
    @staticmethod
    def Dim() -> int:
        """
        return DOF, dimensionality of tangent space
        """
    def K(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        return calibration matrix K
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = 'Cal3') -> str:
        ...
    def aspectRatio(self) -> float:
        """
        aspect ratio
        """
    def dim(self) -> int:
        """
        return DOF, dimensionality of tangent space
        """
    def equals(self, K: Cal3, tol: float) -> bool:
        """
        Check if equal up to specified tolerance.
        """
    def fx(self) -> float:
        """
        focal length x
        """
    def fy(self) -> float:
        """
        focal length y
        """
    def inverse(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Return inverted calibration matrix inv(K)
        """
    def principalPoint(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the principal point
        """
    def print(self, s: str = 'Cal3') -> None:
        """
        print with optional string
        """
    def px(self) -> float:
        """
        image center in x
        """
    def py(self) -> float:
        """
        image center in y
        """
    def skew(self) -> float:
        """
        skew
        """
    def vector(self) -> numpy.ndarray[tuple[typing.Literal[5], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        vectorized form (column-wise)
        """
class Cal3Bundler(Cal3f):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, k1: float, k2: float, u0: float, v0: float) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, k1: float, k2: float, u0: float, v0: float, tol: float) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, K: Cal3Bundler, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def k1(self) -> float:
        """
        distortion parameter k1
        """
    def k2(self) -> float:
        """
        distortion parameter k2
        """
    def localCoordinates(self, T2: Cal3Bundler) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Calculate local coordinates to another calibration.
        """
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3Bundler:
        """
        Update calibration with tangent space delta.
        """
    def serialize(self) -> str:
        ...
class Cal3DS2(Cal3DS2_Base):
    @staticmethod
    def Dim() -> int:
        """
        Return dimensions of calibration manifold object.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float, k1: float, k2: float) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float, k1: float, k2: float, p1: float, p2: float) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        """
        Return dimensions of calibration manifold object.
        """
    def equals(self, K: Cal3DS2, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def localCoordinates(self, T2: Cal3DS2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Given a different calibration, calculate update to obtain it.
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3DS2:
        """
        Given delta vector, update calibration.
        """
    def serialize(self) -> str:
        ...
class Cal3DS2_Base(Cal3):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert (distorted) image coordinates uv to intrinsic coordinates xy.
        """
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert (distorted) image coordinates uv to intrinsic coordinates xy.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def k1(self) -> float:
        """
        First distortion coefficient.
        """
    def k2(self) -> float:
        """
        Second distortion coefficient.
        """
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    def serialize(self) -> str:
        ...
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert intrinsic coordinates xy to (distorted) image coordinates uv 
        p: point in intrinsic coordinates
        Returns: point in (distorted) image coordinates
        """
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert intrinsic coordinates xy to (distorted) image coordinates uv 
        p: point in intrinsic coordinates
        Dcal: optional 2*9 Jacobian wrpt
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates
        Returns: point in (distorted) image coordinates
        """
class Cal3Fisheye(Cal3):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float, k1: float, k2: float, k3: float, k4: float, tol: float = 1e-05) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = 'Cal3Fisheye') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert (distorted) image coordinates [u;v] to intrinsic coordinates [x_i, y_i]. 
        p: point in image coordinates
        Returns: point in intrinsic coordinates
        """
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert (distorted) image coordinates [u;v] to intrinsic coordinates [x_i, y_i]. 
        p: point in image coordinates
        Dcal: optional 2*9 Jacobian wrpt intrinsic parameters
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates (xi, yi)
        Returns: point in intrinsic coordinates
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, K: Cal3Fisheye, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def k1(self) -> float:
        """
        First distortion coefficient.
        """
    def k2(self) -> float:
        """
        Second distortion coefficient.
        """
    def k3(self) -> float:
        """
        First tangential distortion coefficient.
        """
    def k4(self) -> float:
        """
        Second tangential distortion coefficient.
        """
    def localCoordinates(self, T2: Cal3Fisheye) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Given a different calibration, calculate update to obtain it.
        """
    def print(self, s: str = 'Cal3Fisheye') -> None:
        """
        print with optional string
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3Fisheye:
        """
        Given delta vector, update calibration.
        """
    def serialize(self) -> str:
        ...
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert intrinsic coordinates [x_i; y_i] to (distorted) image coordinates [u; v] 
        p: point in intrinsic coordinates
        Returns: point in (distorted) image coordinates
        """
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert intrinsic coordinates [x_i; y_i] to (distorted) image coordinates [u; v] 
        p: point in intrinsic coordinates
        Dcal: optional 2*9 Jacobian wrpt intrinsic parameters
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates (xi, yi)
        Returns: point in (distorted) image coordinates
        """
class Cal3Unified(Cal3DS2_Base):
    @staticmethod
    def Dim() -> int:
        """
        Return dimensions of calibration manifold object.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float, k1: float, k2: float) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float, k1: float, k2: float, p1: float, p2: float, xi: float) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Conver a pixel coordinate to ideal coordinate.
        """
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Conver a pixel coordinate to ideal coordinate.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        """
        Return dimensions of calibration manifold object.
        """
    def equals(self, K: Cal3Unified, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def localCoordinates(self, T2: Cal3Unified) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Given a different calibration, calculate update to obtain it.
        """
    def nPlaneToSpace(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert a normalized unit plane point to 3D space.
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3Unified:
        """
        Given delta vector, update calibration.
        """
    def serialize(self) -> str:
        ...
    def spaceToNPlane(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert a 3D point to normalized unit plane.
        """
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert intrinsic coordinates xy to image coordinates uv 
        p: point in intrinsic coordinates
        Returns: point in image coordinates
        """
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert intrinsic coordinates xy to image coordinates uv 
        p: point in intrinsic coordinates
        Dcal: optional 2*10 Jacobian wrpt
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates
        Returns: point in image coordinates
        """
    def xi(self) -> float:
        """
        mirror parameter
        """
class Cal3_S2(Cal3):
    @staticmethod
    def Dim() -> int:
        """
        return DOF, dimensionality of tangent space
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, fov: float, w: int, h: int) -> None:
        ...
    def __repr__(self, s: str = 'Cal3_S2') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert image coordinates uv to intrinsic coordinates xy. 
        p: point in image coordinates
        Returns: point in intrinsic coordinates
        """
    @typing.overload
    def calibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert image coordinates uv to intrinsic coordinates xy. 
        p: point in image coordinates
        Dcal: optional 2*5 Jacobian wrpt
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates
        Returns: point in intrinsic coordinates
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, K: Cal3_S2, tol: float) -> bool:
        """
        Check if equal up to specified tolerance.
        """
    def localCoordinates(self, T2: Cal3_S2) -> numpy.ndarray[tuple[typing.Literal[5], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Unretraction for the calibration.
        """
    def print(self, s: str = 'Cal3_S2') -> None:
        """
        print with optional string
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3_S2:
        """
        Given 5-dim tangent vector, create new calibration.
        """
    def serialize(self) -> str:
        ...
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves. 
        p: point in intrinsic coordinates
        Returns: point in image coordinates
        """
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves. 
        p: point in intrinsic coordinates
        Dcal: optional 2*5 Jacobian wrpt
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates
        Returns: point in image coordinates
        """
class Cal3_S2Stereo(Cal3):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, fy: float, s: float, u0: float, v0: float, b: float) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def baseline(self) -> float:
        """
        return baseline
        """
    def equals(self, other: Cal3_S2Stereo, tol: float) -> bool:
        """
        Check if equal up to specified tolerance.
        """
    def localCoordinates(self, T2: Cal3_S2Stereo) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Unretraction for the calibration.
        """
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3_S2Stereo:
        """
        Given 6-dim tangent vector, create new calibration.
        """
    def vector(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        vectorized form (column-wise)
        """
class Cal3f(Cal3):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, fx: float, u0: float, v0: float) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def calibrate(self, pi: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert a pixel coordinate to ideal coordinate xy. 
        pi: point in image coordinates
        Returns: point in intrinsic coordinates
        """
    @typing.overload
    def calibrate(self, pi: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Convert a pixel coordinate to ideal coordinate xy. 
        pi: point in image coordinates
        Dcal: optional 2*1 Jacobian wrpt focal length
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates
        Returns: point in intrinsic coordinates
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, K: Cal3f, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def f(self) -> float:
        """
        focal length
        """
    def localCoordinates(self, T2: Cal3f) -> numpy.ndarray[tuple[typing.Literal[1], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Calculate local coordinates to another calibration.
        """
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Cal3f:
        """
        Update calibration with tangent space delta.
        """
    def serialize(self) -> str:
        ...
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        : convert intrinsic coordinates xy to image coordinates uv Version of uncalibrate with derivatives 
        p: point in intrinsic coordinates
        Returns: point in image coordinates
        """
    @typing.overload
    def uncalibrate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        : convert intrinsic coordinates xy to image coordinates uv Version of uncalibrate with derivatives 
        p: point in intrinsic coordinates
        Dcal: optional 2*1 Jacobian wrpt focal length
        Dp: optional 2*2 Jacobian wrpt intrinsic coordinates
        Returns: point in image coordinates
        """
class CalibratedCamera:
    @staticmethod
    def Dim() -> int:
        """
        Deprecated
        """
    @staticmethod
    def Level(pose2: Pose2, height: float) -> CalibratedCamera:
        """
        Create a level camera at the given 2D pose and height. 
        pose2: specifies the location and viewing direction
        height: specifies the height of the camera (along the positive Z-axis) (theta 0 = looking in direction of positive X axis)
        """
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = 'CalibratedCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, pn: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        backproject a 2-dimensional point to a 3-dimensional point at given depth
        """
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        """
        Deprecated
        """
    def equals(self, camera: CalibratedCamera, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: CalibratedCamera) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return canonical coordinate.
        """
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'CalibratedCamera') -> None:
        """
        print
        """
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        DeprecatedUse project2, which is more consistently named across Pinhole cameras
        """
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        DeprecatedUse project2, which is more consistently named across Pinhole cameras
        """
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        """
        Calculate range to a landmark. 
        point: 3D location of landmark
        Returns: range (double)
        """
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        """
        Calculate range to a landmark. 
        point: 3D location of landmark
        Returns: range (double)
        """
    @typing.overload
    def range(self, pose: Pose3) -> float:
        """
        Calculate range to another pose. 
        pose: Other SO(3) pose
        Returns: range (double)
        """
    @typing.overload
    def range(self, point: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, camera: CalibratedCamera) -> float:
        """
        Calculate range to another camera. 
        camera: Other camera
        Returns: range (double)
        """
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> CalibratedCamera:
        """
        move a cameras pose according to d
        """
    def serialize(self) -> str:
        ...
class CameraSetCal3Bundler:
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, s: slice) -> CameraSetCal3Bundler:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ...:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CameraSetCal3Bundler) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None:
        ...
    def __iter__(self) -> typing.Iterator[...]:
        ...
    def __len__(self) -> int:
        ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ...) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: CameraSetCal3Bundler) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: ...) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: CameraSetCal3Bundler) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: int, x: ...) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ...:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: int) -> ...:
        """
        Remove and return the item at index ``i``
        """
class CameraSetCal3DS2:
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, s: slice) -> CameraSetCal3DS2:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ...:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CameraSetCal3DS2) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None:
        ...
    def __iter__(self) -> typing.Iterator[...]:
        ...
    def __len__(self) -> int:
        ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ...) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: CameraSetCal3DS2) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: ...) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: CameraSetCal3DS2) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: int, x: ...) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ...:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: int) -> ...:
        """
        Remove and return the item at index ``i``
        """
class CameraSetCal3Fisheye:
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, s: slice) -> CameraSetCal3Fisheye:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ...:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CameraSetCal3Fisheye) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None:
        ...
    def __iter__(self) -> typing.Iterator[...]:
        ...
    def __len__(self) -> int:
        ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ...) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: CameraSetCal3Fisheye) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: ...) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: CameraSetCal3Fisheye) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: int, x: ...) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ...:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: int) -> ...:
        """
        Remove and return the item at index ``i``
        """
class CameraSetCal3Unified:
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, s: slice) -> CameraSetCal3Unified:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ...:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CameraSetCal3Unified) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None:
        ...
    def __iter__(self) -> typing.Iterator[...]:
        ...
    def __len__(self) -> int:
        ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ...) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: CameraSetCal3Unified) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: ...) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: CameraSetCal3Unified) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: int, x: ...) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ...:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: int) -> ...:
        """
        Remove and return the item at index ``i``
        """
class CameraSetCal3_S2:
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, s: slice) -> CameraSetCal3_S2:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ...:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CameraSetCal3_S2) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None:
        ...
    def __iter__(self) -> typing.Iterator[...]:
        ...
    def __len__(self) -> int:
        ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ...) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: CameraSetCal3_S2) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: ...) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: CameraSetCal3_S2) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: int, x: ...) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ...:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: int) -> ...:
        """
        Remove and return the item at index ``i``
        """
class CameraSetPinholePoseCal3_S2:
    def __init__(self) -> None:
        ...
    def at(self, i: int) -> ...:
        ...
    def push_back(self, cam: ...) -> None:
        ...
class Chebyshev1Basis:
    @staticmethod
    def CalculateWeights(N: int, x: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate Chebyshev Weights on [-1,1] at x up to order N-1 (N values) 
        N: Degree of the polynomial.
        x: Point to evaluate polynomial at.
        """
    @staticmethod
    def WeightMatrix(N: int, X: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
class Chebyshev2:
    @staticmethod
    @typing.overload
    def CalculateWeights(N: int, x: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate Chebyshev Weights on [-1,1] at any x up to order N-1 (N values) These weights implement barycentric interpolation at a specific x. 
        More precisely, f(x) ~ [w0;...;wN] * [f0;...;fN], where the fj are the values of the function f at the Chebyshev points. As such, for a given x we obtain a linear map from parameter vectors f to interpolated values f(x). Optional [a,b] interval can be specified as well.
        """
    @staticmethod
    @typing.overload
    def CalculateWeights(N: int, x: float, a: float, b: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate Chebyshev Weights on [-1,1] at any x up to order N-1 (N values) These weights implement barycentric interpolation at a specific x. 
        More precisely, f(x) ~ [w0;...;wN] * [f0;...;fN], where the fj are the values of the function f at the Chebyshev points. As such, for a given x we obtain a linear map from parameter vectors f to interpolated values f(x). Optional [a,b] interval can be specified as well.
        """
    @staticmethod
    @typing.overload
    def DerivativeWeights(N: int, x: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate derivative of barycentric weights. 
        This is easy and efficient via the DifferentiationMatrix.
        """
    @staticmethod
    @typing.overload
    def DerivativeWeights(N: int, x: float, a: float, b: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate derivative of barycentric weights. 
        This is easy and efficient via the DifferentiationMatrix.
        """
    @staticmethod
    @typing.overload
    def DifferentiationMatrix(N: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Compute D = differentiation matrix, Trefethen00book p.53 When given a parameter vector f of function values at the Chebyshev points, D*f are the values of f'. 
        https://people.maths.ox.ac.uk/trefethen/8all.pdf Theorem 8.4
        """
    @staticmethod
    @typing.overload
    def DifferentiationMatrix(N: int, a: float, b: float) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Compute D = differentiation matrix, for interval [a,b].
        """
    @staticmethod
    @typing.overload
    def DoubleIntegrationWeights(N: int) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Calculate Double Clenshaw-Curtis integration weights We compute them as W * P, where W are the Clenshaw-Curtis weights and P is the integration matrix.
        """
    @staticmethod
    @typing.overload
    def DoubleIntegrationWeights(N: int, a: float, b: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Calculate Double Clenshaw-Curtis integration weights, for interval [a,b].
        """
    @staticmethod
    @typing.overload
    def IntegrationMatrix(N: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        IntegrationMatrix returns the (N+1)×(N+1) matrix P such that for any f, F = P * f recovers F (the antiderivative) satisfying f = D * F and F(0)=0.
        """
    @staticmethod
    @typing.overload
    def IntegrationMatrix(N: int, a: float, b: float) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        IntegrationMatrix returns the (N+1)×(N+1) matrix P for interval [a,b].
        """
    @staticmethod
    @typing.overload
    def IntegrationWeights(N: int) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Calculate Clenshaw-Curtis integration weights. 
        Trefethen00book, pg 128, clencurt.m Note that N in clencurt.m is 1 less than our N
        """
    @staticmethod
    @typing.overload
    def IntegrationWeights(N: int, a: float, b: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Calculate Clenshaw-Curtis integration weights, for interval [a,b].
        """
    @staticmethod
    @typing.overload
    def Point(N: int, j: int) -> float:
        """
        Specific Chebyshev point, within [-1,1] interval. 
        N: The degree of the polynomial
        j: The index of the Chebyshev point
        Returns: double
        """
    @staticmethod
    @typing.overload
    def Point(N: int, j: int, a: float, b: float) -> float:
        """
        Specific Chebyshev point, within [a,b] interval. 
        N: The degree of the polynomial
        j: The index of the Chebyshev point
        a: Lower bound of interval (default: -1)
        b: Upper bound of interval (default: 1)
        Returns: double
        """
    @staticmethod
    @typing.overload
    def Points(N: int) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        All Chebyshev points.
        """
    @staticmethod
    @typing.overload
    def Points(N: int, a: float, b: float) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        All Chebyshev points, within [a,b] interval.
        """
    @staticmethod
    @typing.overload
    def WeightMatrix(N: int, X: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    @typing.overload
    def WeightMatrix(N: int, X: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], a: float, b: float) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
class Chebyshev2Basis:
    @staticmethod
    def CalculateWeights(N: int, x: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate Chebyshev Weights on [-1,1] at any x up to order N-1 (N values). 
        N: Degree of the polynomial.
        x: Point to evaluate polynomial at.
        """
    @staticmethod
    def WeightMatrix(N: int, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
class CombinedImuFactor(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, pose_i: int, vel_i: int, pose_j: int, vel_j: int, bias_i: int, bias_j: int, CombinedPreintegratedMeasurements: PreintegratedCombinedMeasurements) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def evaluateError(self, pose_i: Pose3, vel_i: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], pose_j: Pose3, vel_j: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], bias_i: imuBias.ConstantBias, bias_j: imuBias.ConstantBias) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def preintegratedMeasurements(self) -> PreintegratedCombinedMeasurements:
        """
        Access the preintegrated measurements.
        """
    def serialize(self) -> str:
        ...
class ComponentDerivativeFactorChebyshev1Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class ComponentDerivativeFactorChebyshev2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class ComponentDerivativeFactorChebyshev2Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class ComponentDerivativeFactorFourierBasis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, P: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class ConjugateGradientParameters(IterativeOptimizationParameters):
    epsilon_abs: float
    epsilon_rel: float
    maxIterations: int
    minIterations: int
    reset: int
    def __init__(self) -> None:
        ...
class ConstantTwistScenario(Scenario):
    @typing.overload
    def __init__(self, w: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, w: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], nTb0: Pose3) -> None:
        ...
class ConstantVelocityFactor(NonlinearFactor):
    def __init__(self, i: int, j: int, dt: float, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, x1: NavState, x2: NavState) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class CustomFactor(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, noiseModel: noiseModel.Base, keys: list[int], errorFunction: typing.Callable[[CustomFactor, Values, JacobianVector], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class DSFMapIndexPair:
    def __init__(self) -> None:
        ...
    def find(self, key: IndexPair) -> IndexPair:
        ...
    def merge(self, x: IndexPair, y: IndexPair) -> None:
        ...
    def sets(self) -> dict[IndexPair, set[IndexPair]]:
        ...
class DecisionTreeFactor(DiscreteFactor):
    def __call__(self, arg0: DiscreteValues) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], spec: list[float]) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], table: str) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, table: list[float]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, table: str) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[tuple[int, int]], table: list[float]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[tuple[int, int]], table: str) -> None:
        ...
    @typing.overload
    def __init__(self, c: ...) -> None:
        ...
    def __mul__(self, arg0: DecisionTreeFactor) -> DecisionTreeFactor:
        ...
    def __repr__(self, s: str = 'DecisionTreeFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __truediv__(self, arg0: DecisionTreeFactor) -> DecisionTreeFactor:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as html table. 
        keyFormatter: GTSAM-style Key formatter.
        names: optional, category names corresponding to choices.
        Returns: std::string a html string.
        """
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as markdown table. 
        keyFormatter: GTSAM-style Key formatter.
        names: optional, category names corresponding to choices.
        Returns: std::string a markdown string.
        """
    @typing.overload
    def cardinality(self, j: int) -> int:
        ...
    @typing.overload
    def cardinality(self, j: int) -> int:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., showZero: bool = True) -> str:
        """
        output to graphviz format string
        """
    def enumerate(self) -> list[tuple[DiscreteValues, float]]:
        """
        Enumerate all values into a map from values to double.
        """
    def equals(self, other: DecisionTreeFactor, tol: float = 1e-09) -> bool:
        """
        equality
        """
    @typing.overload
    def max(self, nrFrontals: int) -> DiscreteFactor:
        """
        Create new factor by maximizing over all values with the same separator.
        """
    @typing.overload
    def max(self, keys: Ordering) -> DiscreteFactor:
        """
        Create new factor by maximizing over all values with the same separator.
        """
    def print(self, s: str = 'DecisionTreeFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def sum(self, nrFrontals: int) -> DiscreteFactor:
        """
        Create new factor by summing all values with the same separator values.
        """
    @typing.overload
    def sum(self, keys: Ordering) -> DiscreteFactor:
        """
        Create new factor by summing all values with the same separator values.
        """
class DegeneracyMode:
    """
    Members:
    
      IGNORE_DEGENERACY
    
      ZERO_ON_DEGENERACY
    
      HANDLE_INFINITY
    """
    HANDLE_INFINITY: typing.ClassVar[DegeneracyMode]  # value = <DegeneracyMode.HANDLE_INFINITY: 2>
    IGNORE_DEGENERACY: typing.ClassVar[DegeneracyMode]  # value = <DegeneracyMode.IGNORE_DEGENERACY: 0>
    ZERO_ON_DEGENERACY: typing.ClassVar[DegeneracyMode]  # value = <DegeneracyMode.ZERO_ON_DEGENERACY: 1>
    __members__: typing.ClassVar[dict[str, DegeneracyMode]]  # value = {'IGNORE_DEGENERACY': <DegeneracyMode.IGNORE_DEGENERACY: 0>, 'ZERO_ON_DEGENERACY': <DegeneracyMode.ZERO_ON_DEGENERACY: 1>, 'HANDLE_INFINITY': <DegeneracyMode.HANDLE_INFINITY: 2>}
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class DerivativeFactorChebyshev1Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class DerivativeFactorChebyshev2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class DerivativeFactorChebyshev2Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class DerivativeFactorFourierBasis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class DiscreteBayesNet:
    def __call__(self, arg0: DiscreteValues) -> float:
        ...
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = 'DiscreteBayesNet\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as html tables.
        """
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as markdown tables.
        """
    @typing.overload
    def add(self, s: DiscreteConditional) -> None:
        ...
    @typing.overload
    def add(self, key: tuple[int, int], spec: str) -> None:
        """
        Add a DiscreteDistribution using a table or a string.
        """
    @typing.overload
    def add(self, key: tuple[int, int], parents: DiscreteKeys, spec: str) -> None:
        ...
    @typing.overload
    def add(self, key: tuple[int, int], parents: list[tuple[int, int]], spec: str) -> None:
        ...
    def at(self, i: int) -> DiscreteConditional:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, bn: DiscreteBayesNet, tol: float = 1e-09) -> bool:
        """
        Check equality.
        """
    def evaluate(self, values: DiscreteValues) -> float:
        ...
    def keys(self) -> ...:
        ...
    def logProbability(self, values: DiscreteValues) -> float:
        ...
    def print(self, s: str = 'DiscreteBayesNet\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def sample(self, rng: MT19937 = None) -> DiscreteValues:
        """
        do ancestral sampling 
        Assumes the Bayes net is reverse topologically sorted, i.e. last conditional will be sampled first. If the Bayes net resulted from eliminating a factor graph, this is true for the elimination ordering. a sampled value for all variables.  Returns: a sampled value for all variables.
        """
    @typing.overload
    def sample(self, given: DiscreteValues, rng: MT19937 = None) -> DiscreteValues:
        """
        do ancestral sampling, given certain variables. 
        Assumes the Bayes net is reverse topologically sorted and that the Bayes net does not contain any conditionals for the given values. given values extended with sampled value for all other variables.  Returns: given values extended with sampled value for all other variables.
        """
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def size(self) -> int:
        ...
class DiscreteBayesTree:
    def __call__(self, arg0: DiscreteValues) -> float:
        ...
    def __getitem__(self, arg0: int) -> DiscreteBayesTreeClique:
        ...
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = 'DiscreteBayesTree\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as html tables.
        """
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as markdown tables.
        """
    @typing.overload
    def addClique(self, clique: DiscreteBayesTreeClique) -> None:
        ...
    @typing.overload
    def addClique(self, clique: DiscreteBayesTreeClique, parent_clique: DiscreteBayesTreeClique) -> None:
        ...
    def clique(self, j: int) -> DiscreteBayesTreeClique:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, other: DiscreteBayesTree, tol: float = 1e-09) -> bool:
        """
        Check equality.
        """
    def evaluate(self, values: DiscreteValues) -> float:
        ...
    def insertRoot(self, subtree: DiscreteBayesTreeClique) -> None:
        ...
    def joint(self, j1: int, j2: int) -> ...:
        ...
    def jointBayesNet(self, j1: int, j2: int) -> DiscreteBayesNet:
        ...
    def marginalFactor(self, key: int) -> DiscreteConditional:
        ...
    def numCachedSeparatorMarginals(self) -> int:
        ...
    def print(self, s: str = 'DiscreteBayesTree\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def size(self) -> int:
        ...
class DiscreteBayesTreeClique:
    def __call__(self, arg0: DiscreteValues) -> float:
        ...
    def __getitem__(self, arg0: int) -> DiscreteBayesTreeClique:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, conditional: DiscreteConditional) -> None:
        ...
    def __repr__(self, s: str = 'DiscreteBayesTreeClique', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def conditional(self) -> DiscreteConditional:
        ...
    def evaluate(self, values: DiscreteValues) -> float:
        ...
    def isRoot(self) -> bool:
        ...
    def nrChildren(self) -> int:
        ...
    def print(self, s: str = 'DiscreteBayesTreeClique', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def printSignature(self, s: str = 'Clique: ', formatter: typing.Callable[[int], str] = ...) -> None:
        """
        print index signature only
        """
class DiscreteCluster:
    factors: DiscreteFactorGraph
    orderedFrontalKeys: Ordering
    def __getitem__(self, arg0: int) -> DiscreteCluster:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def nrChildren(self) -> int:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class DiscreteConditional(DecisionTreeFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, nFrontals: int, f: DecisionTreeFactor) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], spec: str) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], parents: DiscreteKeys, spec: str) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], parents: list[tuple[int, int]], spec: str) -> None:
        ...
    @typing.overload
    def __init__(self, joint: DecisionTreeFactor, marginal: DecisionTreeFactor) -> None:
        ...
    @typing.overload
    def __init__(self, joint: DecisionTreeFactor, marginal: DecisionTreeFactor, orderedKeys: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], parents: DiscreteKeys, table: list[float]) -> None:
        ...
    def __mul__(self, arg0: DiscreteConditional) -> DiscreteConditional:
        ...
    def __repr__(self, s: str = 'Discrete Conditional\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as html table.
        """
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as markdown table.
        """
    def argmax(self, parentsValues: DiscreteValues) -> int:
        """
        Return assignment for single frontal variable that maximizes value. 
        parentsValues: Known assignments for the parents.
        Returns: maximizing assignment for the frontal variable.
        """
    def choose(self, given: DiscreteValues) -> DiscreteConditional:
        """
        < DiscreteValues version 
        restrict to given parent values. Note: does not need be complete set. Examples: P(C|D,E) + . -> P(C|D,E) P(C|D,E) + E -> P(C|D) P(C|D,E) + D -> P(C|E) P(C|D,E) + D,E -> P(C) P(C|D,E) + C -> error! a shared_ptr to a new DiscreteConditional Returns: a shared_ptr to a new
        """
    def equals(self, other: DiscreteConditional, tol: float = 1e-09) -> bool:
        """
        GTSAM-style equals.
        """
    @typing.overload
    def error(self, values: DiscreteValues) -> float:
        ...
    @typing.overload
    def error(self, x: ...) -> float:
        ...
    @typing.overload
    def evaluate(self, values: DiscreteValues) -> float:
        ...
    @typing.overload
    def evaluate(self, x: ...) -> float:
        """
        Calculate probability for HybridValuesx. 
        Dispatches to DiscreteValues version.
        """
    def firstFrontalKey(self) -> int:
        ...
    @typing.overload
    def likelihood(self, frontalValues: DiscreteValues) -> DecisionTreeFactor:
        """
        Convert to a likelihood factor by providing value before bar.
        """
    @typing.overload
    def likelihood(self, value: int) -> DecisionTreeFactor:
        ...
    @typing.overload
    def logProbability(self, values: DiscreteValues) -> float:
        ...
    @typing.overload
    def logProbability(self, x: ...) -> float:
        """
        Log-probability is just -error(x).
        """
    def marginal(self, key: int) -> DiscreteConditional:
        """
        Calculate marginal on given key, no parent case.
        """
    def negLogConstant(self) -> float:
        """
        negLogConstant is just zero, such that -logProbability(x) = -log(evaluate(x)) = error(x) and hence error(x) > 0 for all x. 
        Thus -log(K) for the normalization constant k is 0.
        """
    def nrFrontals(self) -> int:
        ...
    def nrParents(self) -> int:
        ...
    def print(self, s: str = 'Discrete Conditional\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def printSignature(self, s: str = 'Discrete Conditional: ', formatter: typing.Callable[[int], str] = ...) -> None:
        """
        print index signature only
        """
    @typing.overload
    def sample(self, parentsValues: DiscreteValues, rng: MT19937 = None) -> int:
        """
        Sample from conditional, given missing variables Example: std::mt19937_64 rng(42); DiscreteValues given = ...; size_t sample = dc.sample(given, &rng);. 
        parentsValues: Known values of the parents
        rng: Pseudo-Random Number Generator.
        Returns: sample from conditional
        """
    @typing.overload
    def sample(self, value: int, rng: MT19937 = None) -> int:
        ...
    @typing.overload
    def sample(self, rng: MT19937 = None) -> int:
        """
        Sample from conditional, zero parent version Example: std::mt19937_64 rng(42); auto sample = dc.sample(&rng);.
        """
    def sampleInPlace(self, parentsValues: DiscreteValues, rng: MT19937 = None) -> None:
        """
        Sample in place with optional PRNG, stores result in partial solution.
        """
class DiscreteDistribution(DiscreteConditional):
    def __call__(self, arg0: int) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, f: DecisionTreeFactor) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], spec: str) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], spec: list[float]) -> None:
        ...
    def __repr__(self, s: str = 'Discrete Prior\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def pmf(self) -> list[float]:
        """
        We also want to keep the Base version, taking DiscreteValues: 
        Return entire probability mass function.
        """
    def print(self, s: str = 'Discrete Prior\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class DiscreteEliminationTree:
    @typing.overload
    def __init__(self, factorGraph: DiscreteFactorGraph, structure: VariableIndex, order: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, factorGraph: DiscreteFactorGraph, order: Ordering) -> None:
        ...
    def __repr__(self, name: str = 'EliminationTree: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, other: DiscreteEliminationTree, tol: float = 1e-09) -> bool:
        """
        Test whether the tree is equal to another.
        """
    def print(self, name: str = 'EliminationTree: ', formatter: typing.Callable[[int], str] = ...) -> None:
        ...
class DiscreteFactor(Factor):
    def __call__(self, arg0: DiscreteValues) -> float:
        ...
    def __repr__(self, s: str = 'DiscreteFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, lf: DiscreteFactor, tol: float = 1e-09) -> bool:
        """
        equals
        """
    def print(self, s: str = 'DiscreteFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class DiscreteFactorGraph:
    def __call__(self, arg0: DiscreteValues) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, bayesNet: DiscreteBayesNet) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_html_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as html tables. 
        keyFormatter: GTSAM-style Key formatter.
        names: optional, a map from Key to category names.
        Returns: std::string a (potentially long) html string.
        """
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def _repr_markdown_(self, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
        """
        Render as markdown tables. 
        keyFormatter: GTSAM-style Key formatter.
        names: optional, a map from Key to category names.
        Returns: std::string a (potentially long) markdown string.
        """
    @typing.overload
    def add(self, j: tuple[int, int], spec: str) -> None:
        ...
    @typing.overload
    def add(self, j: tuple[int, int], spec: list[float]) -> None:
        ...
    @typing.overload
    def add(self, keys: DiscreteKeys, spec: str) -> None:
        ...
    @typing.overload
    def add(self, keys: list[tuple[int, int]], spec: str) -> None:
        ...
    @typing.overload
    def add(self, keys: list[tuple[int, int]], spec: list[float]) -> None:
        ...
    def at(self, i: int) -> DiscreteFactor:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    @typing.overload
    def eliminateMultifrontal(self, type: Ordering.OrderingType = Ordering.OrderingType.COLAMD) -> DiscreteBayesTree:
        ...
    @typing.overload
    def eliminateMultifrontal(self, type: Ordering.OrderingType, function: typing.Callable[[DiscreteFactorGraph, Ordering], tuple[DiscreteConditional, DiscreteFactor]]) -> DiscreteBayesTree:
        ...
    @typing.overload
    def eliminateMultifrontal(self, ordering: Ordering) -> DiscreteBayesTree:
        ...
    @typing.overload
    def eliminateMultifrontal(self, ordering: Ordering, function: typing.Callable[[DiscreteFactorGraph, Ordering], tuple[DiscreteConditional, DiscreteFactor]]) -> DiscreteBayesTree:
        ...
    @typing.overload
    def eliminatePartialMultifrontal(self, ordering: Ordering) -> tuple[DiscreteBayesTree, DiscreteFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialMultifrontal(self, ordering: Ordering, function: typing.Callable[[DiscreteFactorGraph, Ordering], tuple[DiscreteConditional, DiscreteFactor]]) -> tuple[DiscreteBayesTree, DiscreteFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialSequential(self, ordering: Ordering) -> tuple[DiscreteBayesNet, DiscreteFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialSequential(self, ordering: Ordering, function: typing.Callable[[DiscreteFactorGraph, Ordering], tuple[DiscreteConditional, DiscreteFactor]]) -> tuple[DiscreteBayesNet, DiscreteFactorGraph]:
        ...
    @typing.overload
    def eliminateSequential(self, type: Ordering.OrderingType = Ordering.OrderingType.COLAMD) -> DiscreteBayesNet:
        ...
    @typing.overload
    def eliminateSequential(self, type: Ordering.OrderingType, function: typing.Callable[[DiscreteFactorGraph, Ordering], tuple[DiscreteConditional, DiscreteFactor]]) -> DiscreteBayesNet:
        ...
    @typing.overload
    def eliminateSequential(self, ordering: Ordering) -> DiscreteBayesNet:
        ...
    @typing.overload
    def eliminateSequential(self, ordering: Ordering, function: typing.Callable[[DiscreteFactorGraph, Ordering], tuple[DiscreteConditional, DiscreteFactor]]) -> DiscreteBayesNet:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, fg: DiscreteFactorGraph, tol: float = 1e-09) -> bool:
        ...
    def keys(self) -> ...:
        """
        Return the set of variables involved in the factors (set union)
        """
    @typing.overload
    def maxProduct(self, type: Ordering.OrderingType = Ordering.OrderingType.COLAMD) -> DiscreteLookupDAG:
        ...
    @typing.overload
    def maxProduct(self, ordering: Ordering) -> DiscreteLookupDAG:
        """
        Implement the max-product algorithm. 
        ordering: No description provided
        """
    def optimize(self) -> DiscreteValues:
        """
        Find the maximum probable explanation (MPE) by doing max-product.
        """
    def print(self, s: str = '') -> None:
        ...
    def product(self) -> DiscreteFactor:
        """
        return product of all factors as a single factor
        """
    @typing.overload
    def push_back(self, factor: DiscreteFactor) -> None:
        ...
    @typing.overload
    def push_back(self, conditional: DiscreteConditional) -> None:
        ...
    @typing.overload
    def push_back(self, graph: DiscreteFactorGraph) -> None:
        ...
    @typing.overload
    def push_back(self, bayesNet: DiscreteBayesNet) -> None:
        ...
    @typing.overload
    def push_back(self, bayesTree: DiscreteBayesTree) -> None:
        ...
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def size(self) -> int:
        ...
    @typing.overload
    def sumProduct(self, type: Ordering.OrderingType = Ordering.OrderingType.COLAMD) -> DiscreteBayesNet:
        ...
    @typing.overload
    def sumProduct(self, ordering: Ordering) -> DiscreteBayesNet:
        """
        Implement the sum-product algorithm. 
        ordering: No description provided
        """
class DiscreteJunctionTree:
    def __getitem__(self, arg0: int) -> DiscreteCluster:
        ...
    def __init__(self, eliminationTree: DiscreteEliminationTree) -> None:
        ...
    def __repr__(self, name: str = 'JunctionTree: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def nrRoots(self) -> int:
        ...
    def print(self, name: str = 'JunctionTree: ', formatter: typing.Callable[[int], str] = ...) -> None:
        """
        Print the tree to cout.
        """
class DiscreteKeys:
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, n: int) -> tuple[int, int]:
        ...
    def empty(self) -> bool:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        Print the keys and cardinalities.
        """
    def push_back(self, point_pair: tuple[int, int]) -> None:
        ...
    def size(self) -> int:
        ...
class DiscreteLookupDAG:
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = 'DiscreteLookupDAG\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def argmax(self) -> DiscreteValues:
        """
        argmax by back-substitution, optionally given certain variables. 
        Assumes the DAG is reverse topologically sorted, i.e. last conditional will be optimized first and that the DAG does not contain any conditionals for the given variables. If the DAG resulted from eliminating a factor graph, this is true for the elimination ordering. given assignment extended w. optimal assignment for all variables.  Returns: given assignment extended w. optimal assignment for all variables.
        """
    @typing.overload
    def argmax(self, given: DiscreteValues) -> DiscreteValues:
        """
        argmax by back-substitution, optionally given certain variables. 
        Assumes the DAG is reverse topologically sorted, i.e. last conditional will be optimized first and that the DAG does not contain any conditionals for the given variables. If the DAG resulted from eliminating a factor graph, this is true for the elimination ordering. given assignment extended w. optimal assignment for all variables.  Returns: given assignment extended w. optimal assignment for all variables.
        """
    def at(self, i: int) -> DiscreteLookupTable:
        ...
    def empty(self) -> bool:
        ...
    def keys(self) -> ...:
        ...
    def print(self, s: str = 'DiscreteLookupDAG\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def push_back(self, table: DiscreteLookupTable) -> None:
        ...
    def size(self) -> int:
        ...
class DiscreteLookupTable(DiscreteConditional):
    def __init__(self, nFrontals: int, keys: DiscreteKeys, potentials: ...) -> None:
        ...
    def __repr__(self, s: str = 'Discrete Lookup Table: ', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def argmax(self, parentsValues: DiscreteValues) -> int:
        """
        return assignment for single frontal variable that maximizes value. 
        parentsValues: Known assignments for the parents.
        Returns: maximizing assignment for the frontal variable.
        """
    def print(self, s: str = 'Discrete Lookup Table: ', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class DiscreteMarginals:
    def __call__(self, arg0: int) -> DiscreteFactor:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, graph: DiscreteFactorGraph) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def marginalProbabilities(self, key: tuple[int, int]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Compute the marginal of a single variable. 
        key: DiscreteKey of the Variable
        Returns: Vector of marginal probabilities
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class DiscreteSearch:
    @staticmethod
    def FromFactorGraph(factorGraph: DiscreteFactorGraph, ordering: Ordering, buildJunctionTree: bool = False) -> DiscreteSearch:
        """
        Construct from a DiscreteFactorGraph. 
        Internally creates either an elimination tree or a junction tree. The latter incurs more up-front computation but the search itself might be faster. Then again, for the elimination tree, the heuristic will be more fine-grained (more slots). factorGraph: The factor graph to search over.
        ordering: The ordering used to create etree (and maybe jtree).
        buildJunctionTree: Whether to build a junction tree or not.
        """
    @typing.overload
    def __init__(self, etree: DiscreteEliminationTree) -> None:
        ...
    @typing.overload
    def __init__(self, junctionTree: DiscreteJunctionTree) -> None:
        ...
    @typing.overload
    def __init__(self, bayesNet: DiscreteBayesNet) -> None:
        ...
    @typing.overload
    def __init__(self, bayesTree: DiscreteBayesTree) -> None:
        ...
    def __repr__(self, name: str = 'DiscreteSearch: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def lowerBound(self) -> float:
        """
        Return lower bound on the cost-to-go for the entire search.
        """
    def print(self, name: str = 'DiscreteSearch: ', formatter: typing.Callable[[int], str] = ...) -> None:
        """
        Print the tree to cout.
        """
    def run(self, K: int = 1) -> list[DiscreteSearchSolution]:
        """
        Search for the K best solutions. 
        This method performs a search to find the K best solutions for the given DiscreteBayesNet. It uses a priority queue to manage the search nodes, expanding nodes with the smallest bound first. The search continues until all possible nodes have been expanded or pruned. A vector of the K best solutions found during the search.  Returns: A vector of the K best solutions found during the search.
        """
class DiscreteSearchSolution:
    assignment: DiscreteValues
    error: float
    def __init__(self, error: float, assignment: DiscreteValues) -> None:
        ...
class DiscreteValues:
    def __bool__(self) -> bool:
        """
        Check whether the map is nonempty
        """
    @typing.overload
    def __contains__(self, arg0: int) -> bool:
        ...
    @typing.overload
    def __contains__(self, arg0: typing.Any) -> bool:
        ...
    def __delitem__(self, arg0: int) -> None:
        ...
    def __getitem__(self, arg0: int) -> int:
        ...
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator[int]:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        """
        Return the canonical string representation of this map.
        """
    def __setitem__(self, arg0: int, arg1: int) -> None:
        ...
    def items(self) -> typing.ItemsView:
        ...
    def keys(self) -> typing.KeysView:
        ...
    def values(self) -> typing.ValuesView:
        ...
class DoglegOptimizer(NonlinearOptimizer):
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ...) -> None:
        ...
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ..., params: DoglegParams) -> None:
        ...
    def getDelta(self) -> float:
        """
        Access the current trust region radius delta.
        """
class DoglegParams(NonlinearOptimizerParams):
    def __init__(self) -> None:
        ...
    def getDeltaInitial(self) -> float:
        ...
    def getVerbosityDL(self) -> str:
        ...
    def setDeltaInitial(self, deltaInitial: float) -> None:
        ...
    def setVerbosityDL(self, verbosityDL: str) -> None:
        ...
class DotWriter:
    binaryEdges: bool
    boxes: set[int]
    connectKeysToFactor: bool
    factorPositions: dict[int, numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]
    figureHeightInches: float
    figureWidthInches: float
    plotFactorPoints: bool
    positionHints: dict[str, float]
    variablePositions: dict[int, numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]
    def __init__(self, figureWidthInches: float = 5, figureHeightInches: float = 5, plotFactorPoints: bool = True, connectKeysToFactor: bool = True, binaryEdges: bool = True) -> None:
        ...
class DummyPreconditionerParameters(PreconditionerParameters):
    def __init__(self) -> None:
        ...
class EdgeKey:
    @typing.overload
    def __init__(self, i: int, j: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: EdgeKey) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def i(self) -> int:
        """
        Retrieve high 32 bits.
        """
    def j(self) -> int:
        """
        Retrieve low 32 bits.
        """
    def key(self) -> int:
        """
        Cast to Key.
        """
    def print(self, s: str = '') -> None:
        """
        Prints the EdgeKey with an optional prefix string.
        """
class EssentialMatrix:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def FromPose3(_1P2_: Pose3) -> EssentialMatrix:
        """
        Named constructor converting a Pose3 with scale to EssentialMatrix (no scale)
        """
    @staticmethod
    @typing.overload
    def FromPose3(_1P2_: Pose3, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> EssentialMatrix:
        """
        Named constructor converting a Pose3 with scale to EssentialMatrix (no scale)
        """
    def __init__(self, aRb: Rot3, aTb: Unit3) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def dim(self) -> int:
        ...
    def direction(self) -> Unit3:
        """
        Direction.
        """
    def equals(self, other: EssentialMatrix, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def error(self, vA: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], vB: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        """
        epipolar error, algebraic
        """
    def localCoordinates(self, other: EssentialMatrix) -> numpy.ndarray[tuple[typing.Literal[5], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Compute the coordinates in the tangent space.
        """
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Return 3*3 matrix representation.
        """
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    def retract(self, xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> EssentialMatrix:
        """
        Retract delta to manifold.
        """
    def rotation(self) -> Rot3:
        """
        Rotation.
        """
class EssentialMatrixConstraint(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, measuredE: EssentialMatrix, model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, expected: EssentialMatrixConstraint, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, p1: Pose3, p2: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> EssentialMatrix:
        """
        return the measured
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        implement functions needed for Testable
        print
        """
class EssentialMatrixFactor(NoiseModelFactor):
    def __init__(self, key: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class EssentialMatrixFactor2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, d: float) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class EssentialMatrixFactor3(EssentialMatrixFactor2):
    def __init__(self, key1: int, key2: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], cRb: Rot3, model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, d: float) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class EssentialMatrixFactor4Cal3Bundler(NoiseModelFactor):
    def __init__(self, keyE: int, keyK: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, K: Cal3Bundler) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor4Cal3DS2(NoiseModelFactor):
    def __init__(self, keyE: int, keyK: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, K: Cal3DS2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor4Cal3Fisheye(NoiseModelFactor):
    def __init__(self, keyE: int, keyK: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, K: Cal3Fisheye) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor4Cal3Unified(NoiseModelFactor):
    def __init__(self, keyE: int, keyK: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, K: Cal3Unified) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor4Cal3_S2(NoiseModelFactor):
    def __init__(self, keyE: int, keyK: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, K: Cal3_S2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor4Cal3f(NoiseModelFactor):
    def __init__(self, keyE: int, keyK: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, K: Cal3f) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor5Cal3Bundler(NoiseModelFactor):
    def __init__(self, keyE: int, keyKa: int, keyKb: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, Ka: Cal3Bundler, Kb: Cal3Bundler) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor5Cal3DS2(NoiseModelFactor):
    def __init__(self, keyE: int, keyKa: int, keyKb: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, Ka: Cal3DS2, Kb: Cal3DS2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor5Cal3Fisheye(NoiseModelFactor):
    def __init__(self, keyE: int, keyKa: int, keyKb: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, Ka: Cal3Fisheye, Kb: Cal3Fisheye) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor5Cal3Unified(NoiseModelFactor):
    def __init__(self, keyE: int, keyKa: int, keyKb: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, Ka: Cal3Unified, Kb: Cal3Unified) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor5Cal3_S2(NoiseModelFactor):
    def __init__(self, keyE: int, keyKa: int, keyKb: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, Ka: Cal3_S2, Kb: Cal3_S2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialMatrixFactor5Cal3f(NoiseModelFactor):
    def __init__(self, keyE: int, keyKa: int, keyKb: int, pA: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], pB: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base = None) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, E: EssentialMatrix, Ka: Cal3f, Kb: Cal3f) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class EssentialTransferFactorCal3Bundler(NoiseModelFactor):
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], calibration: Cal3Bundler, model: noiseModel.Base = None) -> None:
        ...
class EssentialTransferFactorCal3_S2(NoiseModelFactor):
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], calibration: Cal3_S2, model: noiseModel.Base = None) -> None:
        ...
class EssentialTransferFactorCal3f(NoiseModelFactor):
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], calibration: Cal3f, model: noiseModel.Base = None) -> None:
        ...
class EssentialTransferFactorKCal3Bundler(NoiseModelFactor):
    @typing.overload
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
    @typing.overload
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, keyK: int, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
class EssentialTransferFactorKCal3_S2(NoiseModelFactor):
    @typing.overload
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
    @typing.overload
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, keyK: int, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
class EssentialTransferFactorKCal3f(NoiseModelFactor):
    @typing.overload
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
    @typing.overload
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, keyK: int, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
class EvaluationFactorChebyshev1Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class EvaluationFactorChebyshev2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class EvaluationFactorChebyshev2Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class EvaluationFactorFourierBasis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class Event:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, t: float, p: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, t: float, x: float, y: float, z: float) -> None:
        ...
    def __repr__(self, s: str) -> str:
        ...
    def height(self) -> float:
        ...
    def location(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str) -> None:
        """
        print with optional string
        """
    def time(self) -> float:
        ...
class ExtendedKalmanFilterConstantBias:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: ..., P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> ...:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> ...:
        ...
class ExtendedKalmanFilterNavState:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: ..., P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> ...:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> ...:
        ...
class ExtendedKalmanFilterPoint2:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class ExtendedKalmanFilterPoint3:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class ExtendedKalmanFilterPose2:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: Pose2, P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> Pose2:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> Pose2:
        ...
class ExtendedKalmanFilterPose3:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: Pose3, P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> Pose3:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> Pose3:
        ...
class ExtendedKalmanFilterRot2:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: Rot2, P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> Rot2:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> Rot2:
        ...
class ExtendedKalmanFilterRot3:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: Rot3, P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> Rot3:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> Rot3:
        ...
class ExtendedKalmanFilterSimilarity2:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: Similarity2, P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> Similarity2:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> Similarity2:
        ...
class ExtendedKalmanFilterSimilarity3:
    def Density(self) -> JacobianFactor:
        ...
    def __init__(self, key_initial: int, x_initial: Similarity3, P_initial: noiseModel.Gaussian) -> None:
        ...
    def predict(self, motionFactor: NoiseModelFactor) -> Similarity3:
        ...
    def update(self, measurementFactor: NoiseModelFactor) -> Similarity3:
        ...
class Factor:
    def __repr__(self, s: str = 'Factor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def empty(self) -> bool:
        """
        Whether the factor is empty (involves zero variables).
        """
    def equals(self, other: Factor, tol: float = 1e-09) -> bool:
        """
        check equality
        """
    def keys(self) -> list[int]:
        """
        Access the factor's involved variable keys.
        """
    def print(self, s: str = 'Factor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def printKeys(self, s: str = '') -> None:
        ...
    def size(self) -> int:
        """
        the number of variables involved in this factor  Returns: the number of variables involved in this factor
        """
class FitBasisChebyshev1Basis:
    @staticmethod
    def LinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> GaussianFactorGraph:
        ...
    @staticmethod
    def NonlinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> NonlinearFactorGraph:
        ...
    def __init__(self, sequence: dict[float, float], model: noiseModel.Base, N: int) -> None:
        ...
    def parameters(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FitBasisChebyshev2:
    @staticmethod
    def LinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> GaussianFactorGraph:
        ...
    @staticmethod
    def NonlinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> NonlinearFactorGraph:
        ...
    def __init__(self, sequence: dict[float, float], model: noiseModel.Base, N: int) -> None:
        ...
    def parameters(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FitBasisChebyshev2Basis:
    @staticmethod
    def LinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> GaussianFactorGraph:
        ...
    @staticmethod
    def NonlinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> NonlinearFactorGraph:
        ...
    def __init__(self, sequence: dict[float, float], model: noiseModel.Base, N: int) -> None:
        ...
    def parameters(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FitBasisFourierBasis:
    @staticmethod
    def LinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> GaussianFactorGraph:
        ...
    @staticmethod
    def NonlinearGraph(sequence: dict[float, float], model: noiseModel.Base, N: int) -> NonlinearFactorGraph:
        ...
    def __init__(self, sequence: dict[float, float], model: noiseModel.Base, N: int) -> None:
        ...
    def parameters(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FixedLagSmoother:
    def __repr__(self, s: str) -> str:
        ...
    def calculateEstimate(self) -> ...:
        """
        Compute an estimate from the incomplete linear delta computed during the last update. 
        This delta is incomplete because it was not updated below wildfire_threshold. If only a single variable is needed, it is faster to call calculateEstimate(const KEY&).
        """
    def equals(self, rhs: FixedLagSmoother, tol: float) -> bool:
        """
        Check if two IncrementalFixedLagSmoother Objects are equal.
        """
    def print(self, s: str) -> None:
        ...
    def smootherLag(self) -> float:
        """
        read the current smoother lag
        """
    def timestamps(self) -> dict[int, float]:
        """
        Access the current set of timestamps associated with each variable.
        """
    @typing.overload
    def update(self, newFactors: NonlinearFactorGraph, newTheta: ..., timestamps: dict[int, float]) -> FixedLagSmootherResult:
        ...
    @typing.overload
    def update(self, newFactors: NonlinearFactorGraph, newTheta: ..., timestamps: dict[int, float], factorsToRemove: list[int]) -> FixedLagSmootherResult:
        """
        Add new factors, updating the solution and relinearizing as needed.
        """
class FixedLagSmootherResult:
    def getError(self) -> float:
        ...
    def getIterations(self) -> int:
        ...
    def getLinearVariables(self) -> int:
        ...
    def getNonlinearVariables(self) -> int:
        ...
class FourierBasis:
    @staticmethod
    def CalculateWeights(N: int, x: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Evaluate Real Fourier Weights of size N in interval [a, b], e.g. 
        N=5 yields bases: 1, cos(x), sin(x), cos(2*x), sin(2*x) N: The degree of the polynomial to use.
        x: The point at which to compute the derivaive weights.
        Returns: Weights
        """
    @staticmethod
    def DerivativeWeights(N: int, x: float) -> numpy.ndarray[tuple[typing.Literal[1], N], numpy.dtype[numpy.float64]]:
        """
        Get weights at a given x that calculate the derivative. 
        N: The degree of the polynomial to use.
        x: The point at which to compute the derivaive weights.
        Returns: Weights
        """
    @staticmethod
    def DifferentiationMatrix(N: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Compute D = differentiation matrix. 
        Given coefficients c of a Fourier series c, D*c are the values of c'.
        """
    @staticmethod
    def WeightMatrix(N: int, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusBetweenFactorPose2(NoiseModelFactor):
    @typing.overload
    def __init__(self, j1: int, j2: int, T12: Pose2) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, T12: Pose2, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Pose2, T2: Pose2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusBetweenFactorPose3(NoiseModelFactor):
    @typing.overload
    def __init__(self, j1: int, j2: int, T12: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, T12: Pose3, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Pose3, T2: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusBetweenFactorRot2(NoiseModelFactor):
    @typing.overload
    def __init__(self, j1: int, j2: int, T12: Rot2) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, T12: Rot2, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Rot2, T2: Rot2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusBetweenFactorRot3(NoiseModelFactor):
    @typing.overload
    def __init__(self, j1: int, j2: int, T12: Rot3) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, T12: Rot3, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Rot3, T2: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusBetweenFactorSO3(NoiseModelFactor):
    @typing.overload
    def __init__(self, j1: int, j2: int, T12: SO3) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, T12: SO3, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: SO3, T2: SO3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusBetweenFactorSO4(NoiseModelFactor):
    @typing.overload
    def __init__(self, j1: int, j2: int, T12: SO4) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, T12: SO4, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: SO4, T2: SO4) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusFactorPose2(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Pose2, T2: Pose2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusFactorPose3(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Pose3, T2: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusFactorRot2(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Rot2, T2: Rot2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusFactorRot3(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: Rot3, T2: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusFactorSO3(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: SO3, T2: SO3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusFactorSO4(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, T1: SO4, T2: SO4) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusPriorPose2(NoiseModelFactor):
    def __init__(self, j: int, M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, g: Pose2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusPriorPose3(NoiseModelFactor):
    def __init__(self, j: int, M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, g: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusPriorRot2(NoiseModelFactor):
    def __init__(self, j: int, M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, g: Rot2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusPriorRot3(NoiseModelFactor):
    def __init__(self, j: int, M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, g: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusPriorSO3(NoiseModelFactor):
    def __init__(self, j: int, M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, g: SO3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FrobeniusPriorSO4(NoiseModelFactor):
    def __init__(self, j: int, M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, g: SO4) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class FundamentalMatrix:
    @staticmethod
    def Dim() -> int:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, U: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]], s: float, V: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, F: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, Ka: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]], E: EssentialMatrix, Kb: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, Ka: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]], aPb: Pose3, Kb: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def dim(self) -> int:
        ...
    def equals(self, other: FundamentalMatrix, tol: float = 1e-09) -> bool:
        """
        Check if the FundamentalMatrix is equal to another within a tolerance.
        """
    def localCoordinates(self, F: FundamentalMatrix) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return local coordinates with respect to another FundamentalMatrix.
        """
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Return the fundamental matrix representation.
        """
    def print(self, s: str = '') -> None:
        ...
    def retract(self, delta: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> FundamentalMatrix:
        """
        Retract the given vector to get a new FundamentalMatrix.
        """
class GPSFactor(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, gpsIn: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measurementIn(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the measurement, in the navigation frame
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class GPSFactor2(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, gpsIn: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: NavState) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measurementIn(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the measurement, in the navigation frame
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class GPSFactor2Arm(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, gpsIn: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], leverArm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: NavState) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measurementIn(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the measurement, in the navigation frame
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class GPSFactor2ArmCalib(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, gpsIn: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: NavState, leverArm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measurementIn(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the measurement, in the navigation frame
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class GPSFactorArm(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, gpsIn: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], leverArm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measurementIn(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the measurement, in the navigation frame
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class GPSFactorArmCalib(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, gpsIn: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: Pose3, leverArm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measurementIn(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        return the measurement, in the navigation frame
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class GaussNewtonOptimizer(NonlinearOptimizer):
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ...) -> None:
        ...
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ..., params: GaussNewtonParams) -> None:
        ...
class GaussNewtonParams(NonlinearOptimizerParams):
    def __init__(self) -> None:
        ...
class GaussianBayesNet:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, conditional: GaussianConditional) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, idx: int) -> GaussianConditional:
        ...
    def back(self) -> GaussianConditional:
        ...
    def backSubstitute(self, gx: VectorValues) -> VectorValues:
        """
        Backsubstitute with a different RHS vector than the one stored in this BayesNet. 
        gy=inv(R*inv(Sigma))*gx
        """
    def backSubstituteTranspose(self, gx: VectorValues) -> VectorValues:
        """
        Transpose backsubstitute with a different RHS vector than the one stored in this BayesNet. 
        gy=inv(L)*gx by solving L*gy=gx. gy=inv(R'*inv(Sigma))*gx gz'*R'=gx', gy = gz.*sigmas
        """
    def determinant(self) -> float:
        """
        Computes the determinant of a GassianBayesNet. 
        bayesNet: The input
        Returns: The determinant
        """
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    def equals(self, bn: GaussianBayesNet, tol: float) -> bool:
        """
        Check equality.
        """
    @typing.overload
    def error(self, x: VectorValues) -> float:
        """
        Sum error over all variables.
        """
    @typing.overload
    def error(self, x: VectorValues) -> float:
        """
        Sum error over all variables.
        """
    def evaluate(self, x: VectorValues) -> float:
        """
        Calculate probability density for given values x: exp(logProbability) where x is the vector of values.
        """
    def exists(self, idx: int) -> bool:
        ...
    def front(self) -> GaussianConditional:
        ...
    def gradient(self, x0: VectorValues) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=x_0} \\left\\Vert \\Sigma^{-1} R x - d \\right\\Vert^2 $, centered around $ x = x_0 $. 
        The gradient is $ R^T(Rx-d) $. x0: The center about which to compute the gradient
        Returns: The gradient as a
        """
    def gradientAtZero(self) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=0} \\left\\Vert \\Sigma^{-1} R x - d \\right\\Vert^2 $, centered around zero. 
        The gradient about zero is $ -R^T d $. See also gradient(const GaussianBayesNet&, const VectorValues&). [output]: g A
        """
    def keyVector(self) -> list[int]:
        ...
    def keys(self) -> ...:
        ...
    def logDeterminant(self) -> float:
        """
        Computes the log of the determinant of a GassianBayesNet. 
        bayesNet: The input
        Returns: The determinant
        """
    def logProbability(self, x: VectorValues) -> float:
        """
        Sum logProbability over all variables.
        """
    def matrix(self) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Return (dense) upper-triangular matrix representation Will return upper-triangular matrix only when using 'ordering' above. 
        In case Bayes net is incomplete zero columns are added to the end.
        """
    @typing.overload
    def optimize(self) -> VectorValues:
        """
        Solve the GaussianBayesNet, i.e. 
        return $ x = R^{-1}*d $, by back-substitution
        """
    @typing.overload
    def optimize(self, given: VectorValues) -> VectorValues:
        """
        Version of optimize for incomplete BayesNet, given missing variables.
        """
    def optimizeGradientSearch(self) -> VectorValues:
        """
        Optimize along the gradient direction, with a closed-form computation to perform the line search. 
        The gradient is computed about $ \\delta x=0 $. This function returns $ \\delta x $ that minimizes a reparametrized problem. The error function of a GaussianBayesNet is \\[ f(\\delta x) = \\frac{1}{2} |R \\delta x - d|^2 = \\frac{1}{2}d^T d - d^T R \\delta x + \\frac{1}{2} \\delta x^T R^T R \\delta x \\] with gradient and Hessian \\[ g(\\delta x) = R^T(R\\delta x - d), \\qquad G(\\delta x) = R^T R. \\] This function performs the line search in the direction of the gradient evaluated at $ g = g(\\delta x = 0) $ with step size $ \\alpha $ that minimizes $ f(\\delta x = \\alpha g) $: \\[ f(\\alpha) = \\frac{1}{2} d^T d + g^T \\delta x + \\frac{1}{2} \\alpha^2 g^T G g \\] Optimizing by setting the derivative to zero yields $ \\hat \\alpha = (-g^T g) / (g^T G g) $. For efficiency, this function evaluates the denominator without computing the Hessian $ G $, returning \\[ \\delta x = \\hat\\alpha g = \\frac{-g^T g}{(R g)^T(R g)} \\]
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def push_back(self, conditional: GaussianConditional) -> None:
        ...
    @typing.overload
    def push_back(self, bayesNet: GaussianBayesNet) -> None:
        ...
    @typing.overload
    def sample(self, given: VectorValues, rng: MT19937 = None) -> VectorValues:
        """
        Sample from an incomplete BayesNet, given missing variables Example: std::mt19937_64 rng(42); VectorValues given = ...; auto sample = gbn.sample(given, &rng);.
        """
    @typing.overload
    def sample(self, rng: MT19937 = None) -> VectorValues:
        """
        Sample using ancestral sampling Example: std::mt19937_64 rng(42); auto sample = gbn.sample(&rng);.
        """
    @typing.overload
    def saveGraph(self, s: str) -> None:
        ...
    @typing.overload
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def size(self) -> int:
        ...
class GaussianBayesTree:
    def __getitem__(self, arg0: int) -> GaussianBayesTreeClique:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: GaussianBayesTree) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def determinant(self) -> float:
        """
        Computes the determinant of a GassianBayesTree, as if the Bayes tree is reorganized into a matrix. 
        A GassianBayesTree is equivalent to an upper triangular matrix, and for an upper triangular matrix determinant is the product of the diagonal elements. Instead of actually multiplying we add the logarithms of the diagonal elements and take the exponent at the end because this is more numerically stable.
        """
    def dot(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, other: GaussianBayesTree, tol: float) -> bool:
        """
        Check equality.
        """
    def error(self, x: VectorValues) -> float:
        """
        0.5 * sum of squared Mahalanobis distances.
        """
    def gradient(self, x0: VectorValues) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=x_0} \\left\\Vert \\Sigma^{-1} R x - d \\right\\Vert^2 $, centered around $ x = x_0 $. 
        The gradient is $ R^T(Rx-d) $. x0: The center about which to compute the gradient
        Returns: The gradient as a
        """
    def gradientAtZero(self) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=0} \\left\\Vert \\Sigma^{-1} R x - d \\right\\Vert^2 $, centered around zero. 
        The gradient about zero is $ -R^T d $. See also gradient(const GaussianBayesNet&, const VectorValues&). A VectorValues storing the gradient.  Returns: A
        """
    def joint(self, key1: int, key2: int) -> GaussianFactorGraph:
        ...
    def jointBayesNet(self, key1: int, key2: int) -> GaussianBayesNet:
        ...
    def logDeterminant(self) -> float:
        """
        Computes the determinant of a GassianBayesTree, as if the Bayes tree is reorganized into a matrix. 
        A GassianBayesTree is equivalent to an upper triangular matrix, and for an upper triangular matrix determinant is the product of the diagonal elements. Instead of actually multiplying we add the logarithms of the diagonal elements and take the exponent at the end because this is more numerically stable.
        """
    def marginalCovariance(self, key: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return the marginal on the requested variable as a covariance matrix. 
        See also marginalFactor().
        """
    def marginalFactor(self, key: int) -> GaussianConditional:
        ...
    def numCachedSeparatorMarginals(self) -> int:
        ...
    def optimize(self) -> VectorValues:
        """
        Recursively optimize the BayesTree to produce a vector solution.
        """
    def optimizeGradientSearch(self) -> VectorValues:
        """
        Optimize along the gradient direction, with a closed-form computation to perform the line search. 
        The gradient is computed about $ \\delta x=0 $. This function returns $ \\delta x $ that minimizes a reparametrized problem. The error function of a GaussianBayesNet is \\[ f(\\delta x) = \\frac{1}{2} |R \\delta x - d|^2 = \\frac{1}{2}d^T d - d^T R \\delta x + \\frac{1}{2} \\delta x^T R^T R \\delta x \\] with gradient and Hessian \\[ g(\\delta x) = R^T(R\\delta x - d), \\qquad G(\\delta x) = R^T R. \\] This function performs the line search in the direction of the gradient evaluated at $ g = g(\\delta x = 0) $ with step size $ \\alpha $ that minimizes $ f(\\delta x = \\alpha g) $: \\[ f(\\alpha) = \\frac{1}{2} d^T d + g^T \\delta x + \\frac{1}{2} \\alpha^2 g^T G g \\] Optimizing by setting the derivative to zero yields $ \\hat \\alpha = (-g^T g) / (g^T G g) $. For efficiency, this function evaluates the denominator without computing the Hessian $ G $, returning \\[ \\delta x = \\hat\\alpha g = \\frac{-g^T g}{(R g)^T(R g)} \\]
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def roots(self) -> list[GaussianBayesTreeClique]:
        ...
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def size(self) -> int:
        ...
class GaussianBayesTreeClique:
    def __getitem__(self, arg0: int) -> GaussianBayesTreeClique:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, conditional: GaussianConditional) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def conditional(self) -> GaussianConditional:
        ...
    def deleteCachedShortcuts(self) -> None:
        ...
    def equals(self, other: GaussianBayesTreeClique, tol: float) -> bool:
        ...
    def isRoot(self) -> bool:
        ...
    def nrChildren(self) -> int:
        ...
    def numCachedSeparatorMarginals(self) -> int:
        ...
    def parent(self) -> GaussianBayesTreeClique:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def treeSize(self) -> int:
        ...
class GaussianConditional(JacobianFactor):
    @staticmethod
    @typing.overload
    def FromMeanAndStddev(key: int, mu: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], sigma: float) -> GaussianConditional:
        """
        Construct from mean mu and standard deviation sigma.
        """
    @staticmethod
    @typing.overload
    def FromMeanAndStddev(key: int, A: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], parent: int, b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], sigma: float) -> GaussianConditional:
        """
        Construct from conditional mean A1 p1 + b and standard deviation.
        """
    @staticmethod
    @typing.overload
    def FromMeanAndStddev(key: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], parent1: int, A2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], parent2: int, b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], sigma: float) -> GaussianConditional:
        """
        Construct from conditional mean A1 p1 + A2 p2 + b and standard deviation sigma.
        """
    def R(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return a view of the upper-triangular R block of the conditional.
        """
    def S(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Get a view of the parent blocks.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], sigmas: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], name1: int, S: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], sigmas: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], name1: int, S: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], name2: int, T: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], sigmas: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, terms: list[tuple[int, numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]]], nrFrontals: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], sigmas: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], name1: int, S: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], name1: int, S: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], name2: int, T: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], nrFrontals: int, augmentedMatrix: ...) -> None:
        ...
    def __repr__(self, s: str = 'GaussianConditional', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def d(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get a view of the r.h.s. 
        vector d
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, cg: GaussianConditional, tol: float) -> bool:
        """
        equals function
        """
    @typing.overload
    def error(self, c: VectorValues) -> float:
        ...
    @typing.overload
    def error(self, x: ...) -> float:
        ...
    @typing.overload
    def evaluate(self, x: VectorValues) -> float:
        """
        Calculate probability density for given values x: exp(logProbability(x)) == exp(-GaussianFactor::error(x)) / sqrt((2*pi)^n*det(Sigma)) where x is the vector of values, and Sigma is the covariance matrix.
        """
    @typing.overload
    def evaluate(self, x: ...) -> float:
        """
        Calculate probability for HybridValuesx. 
        Simply dispatches to VectorValues version.
        """
    def firstFrontalKey(self) -> int:
        ...
    @typing.overload
    def likelihood(self, frontalValues: VectorValues) -> JacobianFactor:
        """
        Convert to a likelihood factor by providing value before bar.
        """
    @typing.overload
    def likelihood(self, frontal: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> JacobianFactor:
        """
        Single variable version of likelihood.
        """
    @typing.overload
    def logProbability(self, x: VectorValues) -> float:
        """
        Calculate log-probability log(evaluate(x)) for given values x: -error(x) - 0.5 * n*log(2*pi) - 0.5 * log det(Sigma) where x is the vector of values, and Sigma is the covariance matrix. 
        This differs from error as it is log, not negative log, and it includes the normalization constant.
        """
    @typing.overload
    def logProbability(self, x: ...) -> float:
        """
        Calculate log-probability log(evaluate(x)) for HybridValuesx. 
        Simply dispatches to VectorValues version.
        """
    def negLogConstant(self) -> float:
        """
        Return the negative log of the normalization constant. 
        normalization constant k = 1.0 / sqrt((2*pi)^n*det(Sigma)) -log(k) = 0.5 * n*log(2*pi) + 0.5 * log det(Sigma) double  Returns: double
        """
    def print(self, s: str = 'GaussianConditional', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def sample(self, rng: MT19937 = None) -> VectorValues:
        """
        Sample from conditional, zero parent version Example: std::mt19937_64 rng(42); auto sample = gc.sample(&rng);.
        """
    @typing.overload
    def sample(self, parents: VectorValues, rng: MT19937 = None) -> VectorValues:
        ...
    def serialize(self) -> str:
        ...
    def solve(self, parents: VectorValues) -> VectorValues:
        """
        Solves a conditional Gaussian and writes the solution into the entries of x for each frontal variable of the conditional. 
        The parents are assumed to have already been solved in and their values are read from x. This function works for multiple frontal variables. Given the Gaussian conditional with log likelihood $ |R x_f - (d - S x_s)|^2 $, where $ f $ are the frontal variables and $ s $ are the separator variables of this conditional, this solve function computes $ x_f = R^{-1} (d - S x_s) $ using back-substitution. parents: No description provided
        """
    def solveOtherRHS(self, parents: VectorValues, rhs: VectorValues) -> VectorValues:
        ...
    def solveTransposeInPlace(self, gy: VectorValues) -> None:
        """
        Performs transpose backsubstition in place on values.
        """
class GaussianDensity(GaussianConditional):
    @staticmethod
    def FromMeanAndStddev(key: int, mean: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], sigma: float) -> GaussianDensity:
        """
        Construct using a mean and standard deviation.
        """
    def __init__(self, key: int, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], sigmas: noiseModel.Diagonal) -> None:
        ...
    def __repr__(self, s: str = 'GaussianDensity', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def covariance(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Covariance matrix $ \\Sigma = (R^T R)^{-1} $.
        """
    def equals(self, cg: GaussianDensity, tol: float) -> bool:
        ...
    def mean(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Mean $ \\mu = R^{-1} d $.
        """
    def print(self, s: str = 'GaussianDensity', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class GaussianEliminationTree:
    @typing.overload
    def __init__(self, factorGraph: GaussianFactorGraph, structure: VariableIndex, order: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, factorGraph: GaussianFactorGraph, order: Ordering) -> None:
        ...
    def __repr__(self, name: str = 'GaussianEliminationTree: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, other: GaussianEliminationTree, tol: float) -> bool:
        """
        Test whether the tree is equal to another.
        """
    def print(self, name: str = 'GaussianEliminationTree: ', formatter: typing.Callable[[int], str] = ...) -> None:
        ...
class GaussianFactor(Factor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def augmentedInformation(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return the augmented information matrix represented by this GaussianFactor. 
        The augmented information matrix contains the information matrix with an additional column holding the information vector, and an additional row holding the transpose of the information vector. The lower-right entry contains the constant error term (when $ \\delta x = 0 $). The augmented information matrix is described in more detail in HessianFactor, which in fact stores an augmented information matrix.
        """
    def augmentedJacobian(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return a dense $ [ \\;A\\;b\\; ] \\in \\mathbb{R}^{m \\times n+1} $ Jacobian matrix, augmented with b with the noise models baked into A and b. 
        The negative log-likelihood is $ \\frac{1}{2} \\Vert Ax-b \\Vert^2 $. See also GaussianFactorGraph::jacobian and GaussianFactorGraph::sparseJacobian.
        """
    def clone(self) -> GaussianFactor:
        """
        Clone a factor (make a deep copy)
        """
    def equals(self, lf: GaussianFactor, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def error(self, c: VectorValues) -> float:
        ...
    def information(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return the non-augmented information matrix represented by this GaussianFactor.
        """
    def jacobian(self) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Return the dense Jacobian $ A $ and right-hand-side $ b $, with the noise models baked into A and b. 
        The negative log-likelihood is $ \\frac{1}{2} \\Vert Ax-b \\Vert^2 $. See also GaussianFactorGraph::augmentedJacobian and GaussianFactorGraph::sparseJacobian.
        """
    def negate(self) -> GaussianFactor:
        """
        Construct the corresponding anti-factor to negate information stored stored in this factor. 
        a HessianFactor with negated Hessian matrices  Returns: a
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class GaussianFactorGraph:
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, bayesNet: ...) -> None:
        ...
    @typing.overload
    def __init__(self, bayesTree: ...) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def add(self, factor: GaussianFactor) -> None:
        """
        Add a factor by value - makes a copy.
        """
    @typing.overload
    def add(self, b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        """
        Add a null factor.
        """
    @typing.overload
    def add(self, key1: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> None:
        """
        Add a unary factor.
        """
    @typing.overload
    def add(self, key1: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], key2: int, A2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> None:
        """
        Add a binary factor.
        """
    @typing.overload
    def add(self, key1: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], key2: int, A2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], key3: int, A3: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> None:
        """
        Add a ternary factor.
        """
    def at(self, idx: int) -> GaussianFactor:
        ...
    @typing.overload
    def augmentedHessian(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return a dense $ \\Lambda \\in \\mathbb{R}^{n+1 \\times n+1} $ Hessian matrix, augmented with the information vector $ \\eta $. 
        The augmented Hessian is \\[ \\left[ \\begin{array}{ccc} \\Lambda & \\eta \\\\ \\eta^T & c \\end{array} \\right] \\] and the negative log-likelihood is $ \\frac{1}{2} x^T \\Lambda x + \\eta^T x + c $.
        """
    @typing.overload
    def augmentedHessian(self, ordering: Ordering) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return a dense $ \\Lambda \\in \\mathbb{R}^{n+1 \\times n+1} $ Hessian matrix, augmented with the information vector $ \\eta $. 
        The augmented Hessian is \\[ \\left[ \\begin{array}{ccc} \\Lambda & \\eta \\\\ \\eta^T & c \\end{array} \\right] \\] and the negative log-likelihood is $ \\frac{1}{2} x^T \\Lambda x + \\eta^T x + c $.
        """
    @typing.overload
    def augmentedJacobian(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return a dense $ [ \\;A\\;b\\; ] \\in \\mathbb{R}^{m \\times n+1} $ Jacobian matrix, augmented with b with the noise models baked into A and b. 
        The negative log-likelihood is $ \\frac{1}{2} \\Vert Ax-b \\Vert^2 $. See also GaussianFactorGraph::jacobian and GaussianFactorGraph::sparseJacobian.
        """
    @typing.overload
    def augmentedJacobian(self, ordering: Ordering) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return a dense $ [ \\;A\\;b\\; ] \\in \\mathbb{R}^{m \\times n+1} $ Jacobian matrix, augmented with b with the noise models baked into A and b. 
        The negative log-likelihood is $ \\frac{1}{2} \\Vert Ax-b \\Vert^2 $. See also GaussianFactorGraph::jacobian and GaussianFactorGraph::sparseJacobian.
        """
    def clone(self) -> GaussianFactorGraph:
        """
        Clone() performs a deep-copy of the graph, including all of the factors. 
        Cloning preserves null factors so indices for the original graph are still valid for the cloned graph.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    @typing.overload
    def eliminateMultifrontal(self) -> ...:
        ...
    @typing.overload
    def eliminateMultifrontal(self, type: Ordering.OrderingType) -> ...:
        ...
    @typing.overload
    def eliminateMultifrontal(self, ordering: Ordering) -> ...:
        ...
    @typing.overload
    def eliminatePartialMultifrontal(self, ordering: Ordering) -> tuple[..., GaussianFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialMultifrontal(self, keys: list[int]) -> tuple[..., GaussianFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialSequential(self, ordering: Ordering) -> tuple[..., GaussianFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialSequential(self, keys: list[int]) -> tuple[..., GaussianFactorGraph]:
        ...
    @typing.overload
    def eliminateSequential(self) -> ...:
        ...
    @typing.overload
    def eliminateSequential(self, type: Ordering.OrderingType) -> ...:
        ...
    @typing.overload
    def eliminateSequential(self, ordering: Ordering) -> ...:
        ...
    def equals(self, fg: GaussianFactorGraph, tol: float) -> bool:
        ...
    def error(self, x: VectorValues) -> float:
        """
        unnormalized error
        """
    def exists(self, idx: int) -> bool:
        ...
    def gradient(self, x0: VectorValues) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=x_0} \\left\\Vert \\Sigma^{-1} A x - b \\right\\Vert^2 $, centered around $ x = x_0 $. 
        fg: The Jacobian factor graph $(A,b)$
        x0: The center about which to compute the gradient
        Returns: The gradient as a
        """
    def gradientAtZero(self) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=0} \\left\\Vert \\Sigma^{-1} A x - b \\right\\Vert^2 $, centered around zero. 
        fg: The Jacobian factor graph $(A,b)$
        [output]: g A
        Returns: The gradient as a
        """
    @typing.overload
    def hessian(self) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Return the dense Hessian $ \\Lambda $ and information vector $ \\eta $, with the noise models baked in. 
        The negative log-likelihood is {1}{2} x^T  x + ^T x + c. See also GaussianFactorGraph::augmentedHessian.
        """
    @typing.overload
    def hessian(self, ordering: Ordering) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Return the dense Hessian $ \\Lambda $ and information vector $ \\eta $, with the noise models baked in. 
        The negative log-likelihood is {1}{2} x^T  x + ^T x + c. See also GaussianFactorGraph::augmentedHessian.
        """
    @typing.overload
    def jacobian(self) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Return the dense Jacobian $ A $ and right-hand-side $ b $, with the noise models baked into A and b. 
        The negative log-likelihood is $ \\frac{1}{2} \\Vert Ax-b \\Vert^2 $. See also GaussianFactorGraph::augmentedJacobian and GaussianFactorGraph::sparseJacobian.
        """
    @typing.overload
    def jacobian(self, ordering: Ordering) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Return the dense Jacobian $ A $ and right-hand-side $ b $, with the noise models baked into A and b. 
        The negative log-likelihood is $ \\frac{1}{2} \\Vert Ax-b \\Vert^2 $. See also GaussianFactorGraph::augmentedJacobian and GaussianFactorGraph::sparseJacobian.
        """
    def keyVector(self) -> list[int]:
        ...
    def keys(self) -> ...:
        ...
    def marginal(self, key_vector: list[int]) -> GaussianFactorGraph:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, ordering: Ordering) -> ...:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, key_vector: list[int]) -> ...:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, ordering: Ordering, marginalizedVariableOrdering: Ordering) -> ...:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, key_vector: list[int], marginalizedVariableOrdering: Ordering) -> ...:
        ...
    def negate(self) -> GaussianFactorGraph:
        """
        Returns the negation of all factors in this graph - corresponds to antifactors. 
        Will convert all factors to HessianFactors due to negation of information. Cloning preserves null factors so indices for the original graph are still valid for the cloned graph.
        """
    @typing.overload
    def optimize(self) -> VectorValues:
        """
        Solve the factor graph by performing multifrontal variable elimination in COLAMD order using the dense elimination function specified in function (default EliminatePreferCholesky), followed by back-substitution in the Bayes tree resulting from elimination. 
        Is equivalent to calling graph.eliminateMultifrontal()->optimize().
        """
    @typing.overload
    def optimize(self, ordering: Ordering) -> VectorValues:
        """
        Solve the factor graph by performing multifrontal variable elimination in COLAMD order using the dense elimination function specified in function (default EliminatePreferCholesky), followed by back-substitution in the Bayes tree resulting from elimination. 
        Is equivalent to calling graph.eliminateMultifrontal()->optimize().
        """
    def optimizeDensely(self) -> VectorValues:
        """
        Optimize using Eigen's dense Cholesky factorization.
        """
    def optimizeGradientSearch(self) -> VectorValues:
        """
        Optimize along the gradient direction, with a closed-form computation to perform the line search. 
        The gradient is computed about $ \\delta x=0 $. This function returns $ \\delta x $ that minimizes a reparametrized problem. The error function of a GaussianBayesNet is \\[ f(\\delta x) = \\frac{1}{2} |R \\delta x - d|^2 = \\frac{1}{2}d^T d - d^T R \\delta x + \\frac{1}{2} \\delta x^T R^T R \\delta x \\] with gradient and Hessian \\[ g(\\delta x) = R^T(R\\delta x - d), \\qquad G(\\delta x) = R^T R. \\] This function performs the line search in the direction of the gradient evaluated at $ g = g(\\delta x = 0) $ with step size $ \\alpha $ that minimizes $ f(\\delta x = \\alpha g) $: \\[ f(\\alpha) = \\frac{1}{2} d^T d + g^T \\delta x + \\frac{1}{2} \\alpha^2 g^T G g \\] Optimizing by setting the derivative to zero yields $ \\hat \\alpha = (-g^T g) / (g^T G g) $. For efficiency, this function evaluates the denominator without computing the Hessian $ G $, returning \\[ \\delta x = \\hat\\alpha g = \\frac{-g^T g}{(R g)^T(R g)} \\]
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def printErrors(self, x: VectorValues, str: str = 'GaussianFactorGraph: ', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def probPrime(self, c: VectorValues) -> float:
        """
        Unnormalized probability. 
        O(n)
        """
    @typing.overload
    def push_back(self, factor: GaussianFactor) -> None:
        ...
    @typing.overload
    def push_back(self, conditional: ...) -> None:
        ...
    @typing.overload
    def push_back(self, graph: GaussianFactorGraph) -> None:
        ...
    @typing.overload
    def push_back(self, bayesNet: ...) -> None:
        ...
    @typing.overload
    def push_back(self, bayesTree: ...) -> None:
        ...
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def serialize(self) -> str:
        ...
    def size(self) -> int:
        ...
    def sparseJacobian_(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Matrix version of sparseJacobian: generates a 3*m matrix with [i,j,s] entries such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse. 
        Note: i, j are 1-indexed. The standard deviations are baked into A and b
        """
class GaussianISAM:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, bayesTree: GaussianBayesTree) -> None:
        ...
    def __repr__(self, name: str = 'GaussianISAM: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def clear(self) -> None:
        ...
    def marginalFactor(self, key: int) -> GaussianConditional:
        ...
    def optimize(self) -> VectorValues:
        ...
    def optimizeGradientSearch(self) -> VectorValues:
        ...
    def print(self, name: str = 'GaussianISAM: ', formatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def saveGraph(self, s: str) -> None:
        ...
    def update(self, newFactors: GaussianFactorGraph) -> None:
        ...
class GeneralSFMFactor2Cal3Bundler(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, poseKey: int, landmarkKey: int, calibKey: int) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class GeneralSFMFactor2Cal3DS2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, poseKey: int, landmarkKey: int, calibKey: int) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class GeneralSFMFactor2Cal3Fisheye(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, poseKey: int, landmarkKey: int, calibKey: int) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class GeneralSFMFactor2Cal3Unified(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, poseKey: int, landmarkKey: int, calibKey: int) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class GeneralSFMFactor2Cal3_S2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, poseKey: int, landmarkKey: int, calibKey: int) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class GeneralSFMFactor2Cal3f(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, poseKey: int, landmarkKey: int, calibKey: int) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class GeneralSFMFactorCal3Bundler(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorCal3DS2(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorCal3Fisheye(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorCal3Unified(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorCal3_S2(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorPoseCal3Bundler(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorPoseCal3DS2(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorPoseCal3Fisheye(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorPoseCal3Unified(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GeneralSFMFactorPoseCal3_S2(NoiseModelFactor):
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, cameraKey: int, landmarkKey: int) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class GenericProjectionFactorCal3DS2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3DS2) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3DS2, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3DS2, throwCheirality: bool, verboseCheirality: bool) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3DS2, throwCheirality: bool, verboseCheirality: bool, body_P_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def calibration(self) -> Cal3DS2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
    def throwCheirality(self) -> bool:
        ...
    def verboseCheirality(self) -> bool:
        ...
class GenericProjectionFactorCal3Fisheye(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Fisheye) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Fisheye, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Fisheye, throwCheirality: bool, verboseCheirality: bool) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Fisheye, throwCheirality: bool, verboseCheirality: bool, body_P_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def calibration(self) -> Cal3Fisheye:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
    def throwCheirality(self) -> bool:
        ...
    def verboseCheirality(self) -> bool:
        ...
class GenericProjectionFactorCal3Unified(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Unified) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Unified, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Unified, throwCheirality: bool, verboseCheirality: bool) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3Unified, throwCheirality: bool, verboseCheirality: bool, body_P_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def calibration(self) -> Cal3Unified:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
    def throwCheirality(self) -> bool:
        ...
    def verboseCheirality(self) -> bool:
        ...
class GenericProjectionFactorCal3_S2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3_S2) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3_S2, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3_S2, throwCheirality: bool, verboseCheirality: bool) -> None:
        ...
    @typing.overload
    def __init__(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, poseKey: int, pointKey: int, k: Cal3_S2, throwCheirality: bool, verboseCheirality: bool, body_P_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def calibration(self) -> Cal3_S2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
    def throwCheirality(self) -> bool:
        ...
    def verboseCheirality(self) -> bool:
        ...
class GenericStereoFactor3D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, measured: StereoPoint2, noiseModel: noiseModel.Base, poseKey: int, landmarkKey: int, K: Cal3_S2Stereo) -> None:
        ...
    @typing.overload
    def __init__(self, measured: StereoPoint2, noiseModel: noiseModel.Base, poseKey: int, landmarkKey: int, K: Cal3_S2Stereo, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, measured: StereoPoint2, noiseModel: noiseModel.Base, poseKey: int, landmarkKey: int, K: Cal3_S2Stereo, throwCheirality: bool, verboseCheirality: bool) -> None:
        ...
    @typing.overload
    def __init__(self, measured: StereoPoint2, noiseModel: noiseModel.Base, poseKey: int, landmarkKey: int, K: Cal3_S2Stereo, throwCheirality: bool, verboseCheirality: bool, body_P_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def calibration(self) -> Cal3_S2Stereo:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> StereoPoint2:
        ...
    def serialize(self) -> str:
        ...
class GenericValueCal3Bundler(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueCal3DS2(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueCal3Fisheye(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueCal3Unified(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueCal3_S2(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueCalibratedCamera(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueConstantBias(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueEssentialMatrix(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueMatrix(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValuePoint2(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValuePoint3(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValuePose2(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValuePose3(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueRot2(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueRot3(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueStereoPoint2(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GenericValueVector(Value):
    def __getstate__(self) -> tuple:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class GncGaussNewtonOptimizer:
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ..., params: GncGaussNewtonParams) -> None:
        ...
    def getInlierCostThresholds(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getWeights(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def optimize(self) -> ...:
        ...
    def setInlierCostThresholds(self, inth: float) -> None:
        ...
    def setInlierCostThresholdsAtProbability(self, alpha: float) -> None:
        ...
    def setWeights(self, w: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
class GncGaussNewtonParams:
    class Verbosity:
        """
        Members:
        
          SILENT
        
          SUMMARY
        
          MU
        
          WEIGHTS
        
          VALUES
        """
        MU: typing.ClassVar[GncGaussNewtonParams.Verbosity]  # value = <Verbosity.MU: 2>
        SILENT: typing.ClassVar[GncGaussNewtonParams.Verbosity]  # value = <Verbosity.SILENT: 0>
        SUMMARY: typing.ClassVar[GncGaussNewtonParams.Verbosity]  # value = <Verbosity.SUMMARY: 1>
        VALUES: typing.ClassVar[GncGaussNewtonParams.Verbosity]  # value = <Verbosity.VALUES: 4>
        WEIGHTS: typing.ClassVar[GncGaussNewtonParams.Verbosity]  # value = <Verbosity.WEIGHTS: 3>
        __members__: typing.ClassVar[dict[str, GncGaussNewtonParams.Verbosity]]  # value = {'SILENT': <Verbosity.SILENT: 0>, 'SUMMARY': <Verbosity.SUMMARY: 1>, 'MU': <Verbosity.MU: 2>, 'WEIGHTS': <Verbosity.WEIGHTS: 3>, 'VALUES': <Verbosity.VALUES: 4>}
        def __and__(self, other: typing.Any) -> typing.Any:
            ...
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __ge__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __gt__(self, other: typing.Any) -> bool:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __invert__(self) -> typing.Any:
            ...
        def __le__(self, other: typing.Any) -> bool:
            ...
        def __lt__(self, other: typing.Any) -> bool:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __or__(self, other: typing.Any) -> typing.Any:
            ...
        def __rand__(self, other: typing.Any) -> typing.Any:
            ...
        def __repr__(self) -> str:
            ...
        def __ror__(self, other: typing.Any) -> typing.Any:
            ...
        def __rxor__(self, other: typing.Any) -> typing.Any:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        def __xor__(self, other: typing.Any) -> typing.Any:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    baseOptimizerParams: GaussNewtonParams
    knownInliers: list[int]
    knownOutliers: list[int]
    lossType: GncLossType
    maxIterations: int
    muStep: float
    relativeCostTol: float
    verbosity: ...
    weightsTol: float
    @typing.overload
    def __init__(self, baseOptimizerParams: GaussNewtonParams) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    def __repr__(self, str: str = 'GncParams: ') -> str:
        ...
    def print(self, str: str = 'GncParams: ') -> None:
        ...
    def setKnownInliers(self, knownIn: list[int]) -> None:
        ...
    def setKnownOutliers(self, knownOut: list[int]) -> None:
        ...
    def setLossType(self, type: GncLossType) -> None:
        ...
    def setMaxIterations(self, maxIter: int) -> None:
        ...
    def setMuStep(self, step: float) -> None:
        ...
    def setRelativeCostTol(self, value: float) -> None:
        ...
    def setVerbosityGNC(self, value: ...) -> None:
        ...
    def setWeightsTol(self, value: float) -> None:
        ...
class GncLMOptimizer:
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ..., params: GncLMParams) -> None:
        ...
    def getInlierCostThresholds(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getWeights(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def optimize(self) -> ...:
        ...
    def setInlierCostThresholds(self, inth: float) -> None:
        ...
    def setInlierCostThresholdsAtProbability(self, alpha: float) -> None:
        ...
    def setWeights(self, w: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
class GncLMParams:
    class Verbosity:
        """
        Members:
        
          SILENT
        
          SUMMARY
        
          MU
        
          WEIGHTS
        
          VALUES
        """
        MU: typing.ClassVar[GncLMParams.Verbosity]  # value = <Verbosity.MU: 2>
        SILENT: typing.ClassVar[GncLMParams.Verbosity]  # value = <Verbosity.SILENT: 0>
        SUMMARY: typing.ClassVar[GncLMParams.Verbosity]  # value = <Verbosity.SUMMARY: 1>
        VALUES: typing.ClassVar[GncLMParams.Verbosity]  # value = <Verbosity.VALUES: 4>
        WEIGHTS: typing.ClassVar[GncLMParams.Verbosity]  # value = <Verbosity.WEIGHTS: 3>
        __members__: typing.ClassVar[dict[str, GncLMParams.Verbosity]]  # value = {'SILENT': <Verbosity.SILENT: 0>, 'SUMMARY': <Verbosity.SUMMARY: 1>, 'MU': <Verbosity.MU: 2>, 'WEIGHTS': <Verbosity.WEIGHTS: 3>, 'VALUES': <Verbosity.VALUES: 4>}
        def __and__(self, other: typing.Any) -> typing.Any:
            ...
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __ge__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __gt__(self, other: typing.Any) -> bool:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __invert__(self) -> typing.Any:
            ...
        def __le__(self, other: typing.Any) -> bool:
            ...
        def __lt__(self, other: typing.Any) -> bool:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __or__(self, other: typing.Any) -> typing.Any:
            ...
        def __rand__(self, other: typing.Any) -> typing.Any:
            ...
        def __repr__(self) -> str:
            ...
        def __ror__(self, other: typing.Any) -> typing.Any:
            ...
        def __rxor__(self, other: typing.Any) -> typing.Any:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        def __xor__(self, other: typing.Any) -> typing.Any:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    baseOptimizerParams: LevenbergMarquardtParams
    knownInliers: list[int]
    knownOutliers: list[int]
    lossType: GncLossType
    maxIterations: int
    muStep: float
    relativeCostTol: float
    verbosity: ...
    weightsTol: float
    @typing.overload
    def __init__(self, baseOptimizerParams: LevenbergMarquardtParams) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    def __repr__(self, str: str = 'GncParams: ') -> str:
        ...
    def print(self, str: str = 'GncParams: ') -> None:
        ...
    def setKnownInliers(self, knownIn: list[int]) -> None:
        ...
    def setKnownOutliers(self, knownOut: list[int]) -> None:
        ...
    def setLossType(self, type: GncLossType) -> None:
        ...
    def setMaxIterations(self, maxIter: int) -> None:
        ...
    def setMuStep(self, step: float) -> None:
        ...
    def setRelativeCostTol(self, value: float) -> None:
        ...
    def setVerbosityGNC(self, value: ...) -> None:
        ...
    def setWeightsTol(self, value: float) -> None:
        ...
class GncLossType:
    """
    Members:
    
      GM
    
      TLS
    """
    GM: typing.ClassVar[GncLossType]  # value = <GncLossType.GM: 0>
    TLS: typing.ClassVar[GncLossType]  # value = <GncLossType.TLS: 1>
    __members__: typing.ClassVar[dict[str, GncLossType]]  # value = {'GM': <GncLossType.GM: 0>, 'TLS': <GncLossType.TLS: 1>}
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class GraphvizFormatting(DotWriter):
    class Axis:
        """
        Members:
        
          X
        
          Y
        
          Z
        
          NEGX
        
          NEGY
        
          NEGZ
        """
        NEGX: typing.ClassVar[GraphvizFormatting.Axis]  # value = <Axis.NEGX: 3>
        NEGY: typing.ClassVar[GraphvizFormatting.Axis]  # value = <Axis.NEGY: 4>
        NEGZ: typing.ClassVar[GraphvizFormatting.Axis]  # value = <Axis.NEGZ: 5>
        X: typing.ClassVar[GraphvizFormatting.Axis]  # value = <Axis.X: 0>
        Y: typing.ClassVar[GraphvizFormatting.Axis]  # value = <Axis.Y: 1>
        Z: typing.ClassVar[GraphvizFormatting.Axis]  # value = <Axis.Z: 2>
        __members__: typing.ClassVar[dict[str, GraphvizFormatting.Axis]]  # value = {'X': <Axis.X: 0>, 'Y': <Axis.Y: 1>, 'Z': <Axis.Z: 2>, 'NEGX': <Axis.NEGX: 3>, 'NEGY': <Axis.NEGY: 4>, 'NEGZ': <Axis.NEGZ: 5>}
        def __and__(self, other: typing.Any) -> typing.Any:
            ...
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __ge__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __gt__(self, other: typing.Any) -> bool:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __invert__(self) -> typing.Any:
            ...
        def __le__(self, other: typing.Any) -> bool:
            ...
        def __lt__(self, other: typing.Any) -> bool:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __or__(self, other: typing.Any) -> typing.Any:
            ...
        def __rand__(self, other: typing.Any) -> typing.Any:
            ...
        def __repr__(self) -> str:
            ...
        def __ror__(self, other: typing.Any) -> typing.Any:
            ...
        def __rxor__(self, other: typing.Any) -> typing.Any:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        def __xor__(self, other: typing.Any) -> typing.Any:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    mergeSimilarFactors: bool
    paperHorizontalAxis: ...
    paperVerticalAxis: ...
    scale: float
    def __init__(self) -> None:
        ...
class HessianFactor(GaussianFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, factor: GaussianFactor) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, G: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], g: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], f: float) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, mu: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], Sigma: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, G11: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], G12: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], g1: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], G22: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], g2: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], f: float) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, j3: int, G11: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], G12: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], G13: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], g1: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], G22: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], G23: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], g2: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], G33: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], g3: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], f: float) -> None:
        ...
    @typing.overload
    def __init__(self, factors: ...) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def constantTerm(self) -> float:
        """
        Return the constant term $ f $ as described above. 
        The constant term $ f $ Returns: The constant term
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, lf: GaussianFactor, tol: float) -> bool:
        """
        Compare to another factor for testing (implementing Testable)
        """
    def error(self, c: VectorValues) -> float:
        """
        Evaluate the factor error f(x). 
        returns 0.5*[x -1]'H[x -1] (also see constructor documentation)
        """
    def information(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return the non-augmented information matrix represented by this GaussianFactor.
        """
    def linearTerm(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return the complete linear term $ g $ as described above. 
        The linear term $ g $ Returns: The linear term
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def rows(self) -> int:
        """
        Return the number of columns and rows of the Hessian matrix, including the information vector.
        """
    def serialize(self) -> str:
        ...
class HybridBayesNet:
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = 'HybridBayesNet\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, i: int) -> HybridConditional:
        ...
    def choose(self, assignment: DiscreteValues) -> GaussianBayesNet:
        """
        Get the Gaussian Bayes net P(X|M=m) corresponding to a specific assignment m for the discrete variables M. 
        As the hybrid Bayes net defines P(X,M) = P(X|M) P(M), this method returns the posterior p(X|M=m). assignment: The discrete value assignment for the discrete keys.
        Returns: Gaussian posterior P(X|M=m) as a
        """
    def discreteMarginal(self) -> DiscreteBayesNet:
        """
        Get the discrete Bayes Net P(M). 
        As the hybrid Bayes net defines P(X,M) = P(X|M) P(M), this method returns the marginal distribution on the discrete variables. discrete marginal as a DiscreteBayesNet.  Returns: discrete marginal as a
        """
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, fg: HybridBayesNet, tol: float = 1e-09) -> bool:
        """
        GTSAM-style equals.
        """
    def error(self, values: HybridValues) -> float:
        ...
    def evaluate(self, values: HybridValues) -> float:
        """
        Evaluate hybrid probability density for given HybridValues.
        """
    def keys(self) -> ...:
        ...
    def logProbability(self, x: HybridValues) -> float:
        ...
    @typing.overload
    def optimize(self) -> HybridValues:
        """
        Solve the HybridBayesNet by first computing the MPE of all the discrete variables and then optimizing the continuous variables based on the MPE assignment. 
        HybridValues
        """
    @typing.overload
    def optimize(self, assignment: DiscreteValues) -> VectorValues:
        """
        Given the discrete assignment, return the optimized estimate for the selected Gaussian BayesNet. 
        assignment: An assignment of discrete values.
        """
    def print(self, s: str = 'HybridBayesNet\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def push_back(self, s: HybridGaussianConditional) -> None:
        ...
    @typing.overload
    def push_back(self, s: GaussianConditional) -> None:
        ...
    @typing.overload
    def push_back(self, s: DiscreteConditional) -> None:
        ...
    @typing.overload
    def sample(self, given: HybridValues, rng: MT19937 = None) -> HybridValues:
        """
        Sample from an incomplete BayesNet, given missing variables. 
        Example: std::mt19937_64 rng(42); VectorValues given = ...; auto sample = bn.sample(given, &rng); given: No description provided
        rng: The optional pseudo-random number generator.
        """
    @typing.overload
    def sample(self, rng: MT19937 = None) -> HybridValues:
        """
        Sample using ancestral sampling. 
        Example: std::mt19937_64 rng(42); auto sample = bn.sample(&rng); rng: The optional pseudo-random number generator.
        """
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def size(self) -> int:
        ...
    def toFactorGraph(self, measurements: VectorValues) -> ...:
        """
        Convert a hybrid Bayes net to a hybrid Gaussian factor graph by converting all conditionals with instantiated measurements into likelihood factors.
        """
class HybridBayesTree:
    def __getitem__(self, arg0: int) -> HybridBayesTreeClique:
        ...
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = 'HybridBayesTree\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, other: HybridBayesTree, tol: float = 1e-09) -> bool:
        """
        Check equality.
        """
    def optimize(self) -> HybridValues:
        """
        Optimize the hybrid Bayes tree by computing the MPE for the current set of discrete variables and using it to compute the best continuous update delta. 
        HybridValues
        """
    def print(self, s: str = 'HybridBayesTree\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def size(self) -> int:
        ...
class HybridBayesTreeClique:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, conditional: HybridConditional) -> None:
        ...
    def conditional(self) -> HybridConditional:
        ...
    def isRoot(self) -> bool:
        ...
class HybridConditional:
    def __call__(self, arg0: HybridValues) -> float:
        ...
    def __repr__(self, s: str = 'Hybrid Conditional\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def asDiscrete(self) -> DiscreteConditional:
        """
        Return conditional as a DiscreteConditional or specified type T. 
        nullptr if not a DiscreteConditionalDiscreteConditional::shared_ptr Returns: nullptr if not a
        """
    def asGaussian(self) -> GaussianConditional:
        """
        Return HybridConditional as a GaussianConditional. 
        nullptr if not a GaussianConditionalGaussianConditional::shared_ptr otherwise  Returns: nullptr if not a
        """
    def asHybrid(self) -> ...:
        """
        Return HybridConditional as a HybridGaussianConditional. 
        nullptr if not a conditional HybridGaussianConditional::shared_ptr otherwise  Returns: nullptr if not a conditional
        """
    def equals(self, other: HybridConditional, tol: float = 1e-09) -> bool:
        """
        GTSAM-style equals.
        """
    def error(self, values: HybridValues) -> float:
        """
        Return the error of the underlying conditional.
        """
    def evaluate(self, values: HybridValues) -> float:
        """
        Return the probability (or density) of the underlying conditional.
        """
    def inner(self) -> Factor:
        """
        Get the type-erased pointer to the inner type.
        """
    def logProbability(self, values: HybridValues) -> float:
        """
        Return the log-probability (or density) of the underlying conditional.
        """
    def negLogConstant(self) -> float:
        """
        Return the negative log of the normalization constant. 
        This shows up in the error as -(error(x) + negLogConstant) Note this is 0.0 for discrete and hybrid conditionals, but depends on the continuous parameters for Gaussian conditionals.
        """
    def nrFrontals(self) -> int:
        ...
    def nrParents(self) -> int:
        ...
    def print(self, s: str = 'Hybrid Conditional\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class HybridFactor(Factor):
    def __repr__(self, s: str = 'HybridFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def continuousKeys(self) -> list[int]:
        """
        Return only the continuous keys for this factor.
        """
    def discreteKeys(self) -> DiscreteKeys:
        """
        Return the discrete keys for this factor.
        """
    def equals(self, lf: HybridFactor, tol: float = 1e-09) -> bool:
        """
        equals
        """
    def error(self, values: HybridValues) -> float:
        ...
    def isContinuous(self) -> bool:
        """
        True if this is a factor of continuous variables only.
        """
    def isDiscrete(self) -> bool:
        """
        True if this is a factor of discrete variables only.
        """
    def isHybrid(self) -> bool:
        """
        True is this is a Discrete-Continuous factor.
        """
    def nrContinuous(self) -> int:
        """
        Return the number of continuous variables in this factor.
        """
    def print(self, s: str = 'HybridFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class HybridGaussianConditional(HybridFactor):
    @typing.overload
    def __init__(self, discreteParents: DiscreteKeys, conditionals: ..., std: ...) -> None:
        ...
    @typing.overload
    def __init__(self, discreteParent: tuple[int, int], conditionals: list[GaussianConditional]) -> None:
        ...
    def __repr__(self, s: str = 'HybridGaussianConditional\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluate(self, values: HybridValues) -> float:
        """
        Calculate probability density for given values.
        """
    def likelihood(self, frontals: VectorValues) -> HybridGaussianFactor:
        ...
    def logProbability(self, values: HybridValues) -> float:
        """
        Compute the logProbability of this hybrid Gaussian conditional. 
        values: Continuous values and discrete assignment.
        Returns: double
        """
    def print(self, s: str = 'HybridGaussianConditional\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class HybridGaussianFactor(HybridFactor):
    @typing.overload
    def __init__(self, discreteKey: tuple[int, int], factors: list[GaussianFactor]) -> None:
        ...
    @typing.overload
    def __init__(self, discreteKey: tuple[int, int], factorPairs: list[tuple[GaussianFactor, float]]) -> None:
        ...
    def __repr__(self, s: str = 'HybridGaussianFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def print(self, s: str = 'HybridGaussianFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class HybridGaussianFactorGraph:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, bayesNet: HybridBayesNet) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def at(self, i: int) -> Factor:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    @typing.overload
    def eliminateMultifrontal(self) -> HybridBayesTree:
        ...
    @typing.overload
    def eliminateMultifrontal(self, type: Ordering.OrderingType) -> HybridBayesTree:
        ...
    @typing.overload
    def eliminateMultifrontal(self, ordering: Ordering) -> HybridBayesTree:
        ...
    def eliminatePartialMultifrontal(self, ordering: Ordering) -> tuple[HybridBayesTree, HybridGaussianFactorGraph]:
        ...
    def eliminatePartialSequential(self, ordering: Ordering) -> tuple[HybridBayesNet, HybridGaussianFactorGraph]:
        ...
    @typing.overload
    def eliminateSequential(self) -> HybridBayesNet:
        ...
    @typing.overload
    def eliminateSequential(self, type: Ordering.OrderingType) -> HybridBayesNet:
        ...
    @typing.overload
    def eliminateSequential(self, ordering: Ordering) -> HybridBayesNet:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, fg: HybridGaussianFactorGraph, tol: float = 1e-09) -> bool:
        ...
    def error(self, values: HybridValues) -> float:
        ...
    def keys(self) -> ...:
        ...
    def print(self, s: str = '') -> None:
        ...
    def probPrime(self, values: HybridValues) -> float:
        """
        Compute the unnormalized posterior probability for a continuous vector values given a specific assignment. 
        double  Returns: double
        """
    @typing.overload
    def push_back(self, factor: HybridFactor) -> None:
        ...
    @typing.overload
    def push_back(self, conditional: HybridConditional) -> None:
        ...
    @typing.overload
    def push_back(self, graph: HybridGaussianFactorGraph) -> None:
        ...
    @typing.overload
    def push_back(self, bayesNet: HybridBayesNet) -> None:
        ...
    @typing.overload
    def push_back(self, bayesTree: HybridBayesTree) -> None:
        ...
    @typing.overload
    def push_back(self, gmm: HybridGaussianFactor) -> None:
        ...
    @typing.overload
    def push_back(self, factor: DecisionTreeFactor) -> None:
        ...
    @typing.overload
    def push_back(self, factor: TableFactor) -> None:
        ...
    @typing.overload
    def push_back(self, factor: JacobianFactor) -> None:
        ...
    def remove(self, i: int) -> None:
        ...
    def size(self) -> int:
        ...
class HybridNonlinearFactor(HybridFactor):
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> None:
        ...
    @typing.overload
    def __init__(self, discreteKey: tuple[int, int], factors: list[NoiseModelFactor]) -> None:
        ...
    @typing.overload
    def __init__(self, discreteKey: tuple[int, int], factors: list[tuple[NoiseModelFactor, float]]) -> None:
        ...
    def __repr__(self, s: str = 'HybridNonlinearFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def error(self, continuousValues: Values, discreteValues: DiscreteValues) -> float:
        ...
    def linearize(self, continuousValues: Values) -> HybridGaussianFactor:
        """
        Linearize all the continuous factors to get a HybridGaussianFactor.
        """
    def print(self, s: str = 'HybridNonlinearFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print to stdout
        """
class HybridNonlinearFactorGraph:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, graph: HybridNonlinearFactorGraph) -> None:
        ...
    def __repr__(self, s: str = 'HybridNonlinearFactorGraph\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, i: int) -> Factor:
        ...
    def empty(self) -> bool:
        ...
    def keys(self) -> ...:
        ...
    def linearize(self, continuousValues: Values) -> HybridGaussianFactorGraph:
        """
        Linearize all the continuous factors in the HybridNonlinearFactorGraph. 
        continuousValues: Dictionary of continuous values.
        """
    def print(self, s: str = 'HybridNonlinearFactorGraph\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        Print the factor graph.
        """
    @typing.overload
    def push_back(self, factor: HybridFactor) -> None:
        ...
    @typing.overload
    def push_back(self, factor: NonlinearFactor) -> None:
        ...
    @typing.overload
    def push_back(self, factor: DiscreteFactor) -> None:
        ...
    @typing.overload
    def push_back(self, graph: HybridNonlinearFactorGraph) -> None:
        ...
    def remove(self, i: int) -> None:
        ...
    def resize(self, size: int) -> None:
        ...
    def restrict(self, assignment: DiscreteValues) -> HybridNonlinearFactorGraph:
        """
        Restrict all factors in the graph to the given discrete values.
        """
    def size(self) -> int:
        ...
class HybridSmoother:
    def __init__(self, marginalThreshold: float | None = None) -> None:
        ...
    def addConditionals(self, graph: HybridGaussianFactorGraph, hybridBayesNet: HybridBayesNet) -> tuple[HybridGaussianFactorGraph, HybridBayesNet]:
        """
        Add conditionals from previous timestep as part of liquefication. 
        graph: The new factor graph for the current time step.
        hybridBayesNet: The hybrid bayes net containing all conditionals so far.
        ordering: The elimination ordering.
        Returns: std::pair<HybridGaussianFactorGraph, HybridBayesNet>
        """
    def allFactors(self) -> HybridNonlinearFactorGraph:
        """
        Return all the recorded nonlinear factors.
        """
    def fixedValues(self) -> DiscreteValues:
        """
        Return fixed values:
        """
    def gaussianMixture(self, index: int) -> HybridGaussianConditional:
        """
        Get the hybrid Gaussian conditional from the Bayes Net posterior at index. 
        index: Indexing value.
        Returns: HybridGaussianConditional::shared_ptr
        """
    def getOrdering(self, factors: HybridGaussianFactorGraph, newFactorKeys: ...) -> Ordering:
        """
        Get an elimination ordering which eliminates continuous and then discrete. 
        Expects factors to already have the necessary conditionals which were connected to the variables in the newly added factors. Those variables should be in newFactorKeys. factors: All the new factors and connected conditionals.
        newFactorKeys: The keys/variables in the newly added factors.
        """
    def hybridBayesNet(self) -> HybridBayesNet:
        """
        Return the Bayes Net posterior.
        """
    def linearizationPoint(self) -> Values:
        """
        Return the current linearization point.
        """
    def optimize(self) -> HybridValues:
        """
        Optimize the hybrid Bayes Net, taking into accound fixed values.
        """
    def reInitialize(self, hybridBayesNet: HybridBayesNet) -> None:
        """
        Re-initialize the smoother from a new hybrid Bayes Net.
        """
    def relinearize(self, givenOrdering: Ordering | None = None) -> None:
        """
        Relinearize the nonlinear factor graph with the latest stored linearization point. 
        givenOrdering: An optional elimination ordering.
        """
    def update(self, graph: HybridNonlinearFactorGraph, initial: Values, maxNrLeaves: int | None = None, given_ordering: Ordering | None = None) -> None:
        ...
class HybridValues:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, cv: VectorValues, dv: DiscreteValues) -> None:
        ...
    def __repr__(self, s: str = 'HybridValues', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, j: int) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Read/write access to the vector value with key j, throws std::out_of_range if j does not exist.
        """
    def atDiscrete(self, j: int) -> int:
        """
        Read/write access to the discrete value with key j, throws std::out_of_range if j does not exist.
        """
    def continuous(self) -> VectorValues:
        """
        Return the multi-dimensional vector values.
        """
    def discrete(self) -> DiscreteValues:
        """
        Return the discrete values.
        """
    def equals(self, other: HybridValues, tol: float) -> bool:
        """
        equals required by Testable for unit testing
        """
    @typing.overload
    def insert(self, j: int, value: int) -> None:
        """
        Insert a vector value with key j. 
        value: The vector to be inserted.
        j: The index with which the value will be associated.
        """
    @typing.overload
    def insert(self, j: int, value: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        """
        Insert a discrete value with key j. 
        value: The vector to be inserted.
        j: The index with which the value will be associated.
        """
    @typing.overload
    def insert(self, values: VectorValues) -> None:
        """
        Insert all continuous values from values. 
        Throws an invalid_argument exception if any keys to be inserted are already used.
        """
    @typing.overload
    def insert(self, values: DiscreteValues) -> None:
        """
        Insert all discrete values from values. 
        Throws an invalid_argument exception if any keys to be inserted are already used.
        """
    @typing.overload
    def insert(self, values: HybridValues) -> None:
        """
        Insert all values from values. 
        Throws an invalid_argument exception if any keys to be inserted are already used.
        """
    @typing.overload
    def insert_or_assign(self, j: int, value: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        """
        insert_or_assign() , similar to Values.h
        """
    @typing.overload
    def insert_or_assign(self, j: int, value: int) -> None:
        """
        insert_or_assign() , similar to Values.h
        """
    def print(self, s: str = 'HybridValues', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print required by Testable for unit testing
        """
    @typing.overload
    def update(self, values: VectorValues) -> None:
        """
        For all key/value pairs in values, replace continuous values with corresponding keys in this object with those in values. 
        Throws std::out_of_range if any keys in values are not present in this object.
        """
    @typing.overload
    def update(self, values: DiscreteValues) -> None:
        """
        For all key/value pairs in values, replace discrete values with corresponding keys in this object with those in values. 
        Throws std::out_of_range if any keys in values are not present in this object.
        """
    @typing.overload
    def update(self, values: HybridValues) -> None:
        """
        For all key/value pairs in values, replace all values with corresponding keys in this object with those in values. 
        Throws std::out_of_range if any keys in values are not present in this object.
        """
class ISAM2:
    @staticmethod
    @typing.overload
    def update(*args, **kwargs) -> ISAM2Result:
        ...
    @staticmethod
    @typing.overload
    def update(*args, **kwargs) -> ISAM2Result:
        ...
    @staticmethod
    @typing.overload
    def update(*args, **kwargs) -> ISAM2Result:
        """
        Add new factors, updating the solution and relinearizing as needed. 
        Optionally, this function remove existing factors from the system to enable behaviors such as swapping existing factors with new ones. Add new measurements, and optionally new variables, to the current system. This runs a full step of the ISAM2 algorithm, relinearizing and updating the solution as needed, according to the wildfire and relinearize thresholds. newFactors: The new factors to be added to the system
        newTheta: Initialization points for new variables to be added to the system. You must include here all new variables occuring in newFactors (which were not already in the system). There must not be any variables here that do not occur in newFactors, and additionally, variables that were already in the system must not be included here.
        removeFactorIndices: Indices of factors to remove from system
        force_relinearize: Relinearize any variables whose delta magnitude is sufficiently large (Params::relinearizeThreshold), regardless of the relinearization interval (Params::relinearizeSkip).
        constrainedKeys: is an optional map of keys to group labels, such that a variable can be constrained to a particular grouping in the
        noRelinKeys: is an optional set of nonlinear keys that iSAM2 will hold at a constant linearization point, regardless of the size of the linear delta
        extraReelimKeys: is an optional set of nonlinear keys that iSAM2 will re-eliminate, regardless of the size of the linear delta. This allows the provided keys to be reordered.
        Returns: An
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, params: ISAM2Params) -> None:
        ...
    @typing.overload
    def __init__(self, other: ISAM2) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def calculateBestEstimate(self) -> ...:
        """
        Compute an estimate using a complete delta computed by a full back-substitution.
        """
    def calculateEstimate(self) -> ...:
        """
        Compute an estimate from the incomplete linear delta computed during the last update. 
        This delta is incomplete because it was not updated below wildfire_threshold. If only a single variable is needed, it is faster to call calculateEstimate(const KEY&).
        """
    def calculateEstimateCal3Bundler(self, key: int) -> Cal3Bundler:
        ...
    def calculateEstimateCal3DS2(self, key: int) -> Cal3DS2:
        ...
    def calculateEstimateCal3_S2(self, key: int) -> Cal3_S2:
        ...
    def calculateEstimateCal3f(self, key: int) -> Cal3f:
        ...
    def calculateEstimateConstantBias(self, key: int) -> ...:
        ...
    def calculateEstimateEssentialMatrix(self, key: int) -> EssentialMatrix:
        ...
    def calculateEstimateFundamentalMatrix(self, key: int) -> FundamentalMatrix:
        ...
    def calculateEstimateMatrix(self, key: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def calculateEstimatePinholeCameraCal3Bundler(self, key: int) -> PinholeCameraCal3Bundler:
        ...
    def calculateEstimatePinholeCameraCal3Fisheye(self, key: int) -> PinholeCameraCal3Fisheye:
        ...
    def calculateEstimatePinholeCameraCal3Unified(self, key: int) -> PinholeCameraCal3Unified:
        ...
    def calculateEstimatePinholeCameraCal3_S2(self, key: int) -> PinholeCameraCal3_S2:
        ...
    def calculateEstimatePoint2(self, key: int) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calculateEstimatePoint3(self, key: int) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calculateEstimatePose2(self, key: int) -> Pose2:
        ...
    def calculateEstimatePose3(self, key: int) -> Pose3:
        ...
    def calculateEstimateRot2(self, key: int) -> Rot2:
        ...
    def calculateEstimateRot3(self, key: int) -> Rot3:
        ...
    def calculateEstimateSimilarity2(self, key: int) -> Similarity2:
        ...
    def calculateEstimateSimilarity3(self, key: int) -> Similarity3:
        ...
    def calculateEstimateSimpleFundamentalMatrix(self, key: int) -> SimpleFundamentalMatrix:
        ...
    def calculateEstimateVector(self, key: int) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, other: ISAM2, tol: float) -> bool:
        """
        Compare equality.
        """
    @typing.overload
    def error(self, x: VectorValues) -> float:
        """
        Compute the linear error.
        """
    @typing.overload
    def error(self, x: VectorValues) -> float:
        """
        Compute the linear error.
        """
    def getDelta(self) -> VectorValues:
        """
        Access the current delta, computed during the last call to update.
        """
    def getFactorsUnsafe(self) -> NonlinearFactorGraph:
        """
        Access the set of nonlinear factors.
        """
    def getFixedVariables(self) -> ...:
        """
        Access the nonlinear variable index.
        """
    def getLinearizationPoint(self) -> ...:
        """
        Access the current linearization point.
        """
    def getVariableIndex(self) -> VariableIndex:
        """
        Access the nonlinear variable index.
        """
    def gradientAtZero(self) -> VectorValues:
        """
        Compute the gradient of the energy function, $ \\nabla_{x=0} \\left\\Vert \\Sigma^{-1} R x - d \\right\\Vert^2 $, centered around zero. 
        The gradient about zero is $ -R^T d $. See also gradient(const GaussianBayesNet&, const VectorValues&). A VectorValues storing the gradient.  Returns: A
        """
    def marginalCovariance(self, key: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return marginal on any variable as a covariance matrix.
        """
    def params(self) -> ISAM2Params:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def printStats(self) -> None:
        """
        prints out clique statistics
        """
    @typing.overload
    def printStats(self) -> None:
        """
        prints out clique statistics
        """
    @typing.overload
    def saveGraph(self, s: str) -> None:
        ...
    @typing.overload
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def update(self) -> ISAM2Result:
        """
        Add new factors, updating the solution and relinearizing as needed. 
        Optionally, this function remove existing factors from the system to enable behaviors such as swapping existing factors with new ones. Add new measurements, and optionally new variables, to the current system. This runs a full step of the ISAM2 algorithm, relinearizing and updating the solution as needed, according to the wildfire and relinearize thresholds. Returns: An
        """
    @typing.overload
    def update(self, newFactors: NonlinearFactorGraph, newTheta: ...) -> ISAM2Result:
        ...
    @typing.overload
    def update(self, newFactors: NonlinearFactorGraph, newTheta: ..., removeFactorIndices: list[int]) -> ISAM2Result:
        ...
    @typing.overload
    def update(self, newFactors: NonlinearFactorGraph, newTheta: ..., updateParams: ...) -> ISAM2Result:
        """
        Add new factors, updating the solution and relinearizing as needed. 
        Alternative signature of update() (see its documentation above), with all additional parameters in one structure. This form makes easier to keep future API/ABI compatibility if parameters change. newFactors: The new factors to be added to the system
        newTheta: Initialization points for new variables to be added to the system. You must include here all new variables occuring in newFactors (which were not already in the system). There must not be any variables here that do not occur in newFactors, and additionally, variables that were already in the system must not be included here.
        updateParams: Additional parameters to control relinearization, constrained keys, etc.
        Returns: An
        """
    def valueExists(self, key: int) -> bool:
        """
        Check whether variable with given key exists in linearization point.
        """
class ISAM2Clique:
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def gradientContribution(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Access the gradient contribution.
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class ISAM2DoglegParams:
    def __init__(self) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def getAdaptationMode(self) -> str:
        ...
    def getInitialDelta(self) -> float:
        ...
    def getWildfireThreshold(self) -> float:
        ...
    def isVerbose(self) -> bool:
        ...
    def print(self, str: str = '') -> None:
        ...
    def setAdaptationMode(self, adaptationMode: str) -> None:
        ...
    def setInitialDelta(self, initialDelta: float) -> None:
        ...
    def setVerbose(self, verbose: bool) -> None:
        ...
    def setWildfireThreshold(self, wildfireThreshold: float) -> None:
        ...
class ISAM2GaussNewtonParams:
    def __init__(self, _wildfireThreshold: float = 0.001) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def getWildfireThreshold(self) -> float:
        ...
    def print(self, str: str = '') -> None:
        ...
    def setWildfireThreshold(self, wildfireThreshold: float) -> None:
        ...
class ISAM2Params:
    class Factorization:
        """
        Members:
        
          CHOLESKY
        
          QR
        """
        CHOLESKY: typing.ClassVar[ISAM2Params.Factorization]  # value = <Factorization.CHOLESKY: 0>
        QR: typing.ClassVar[ISAM2Params.Factorization]  # value = <Factorization.QR: 1>
        __members__: typing.ClassVar[dict[str, ISAM2Params.Factorization]]  # value = {'CHOLESKY': <Factorization.CHOLESKY: 0>, 'QR': <Factorization.QR: 1>}
        def __and__(self, other: typing.Any) -> typing.Any:
            ...
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __ge__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __gt__(self, other: typing.Any) -> bool:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __invert__(self) -> typing.Any:
            ...
        def __le__(self, other: typing.Any) -> bool:
            ...
        def __lt__(self, other: typing.Any) -> bool:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __or__(self, other: typing.Any) -> typing.Any:
            ...
        def __rand__(self, other: typing.Any) -> typing.Any:
            ...
        def __repr__(self) -> str:
            ...
        def __ror__(self, other: typing.Any) -> typing.Any:
            ...
        def __rxor__(self, other: typing.Any) -> typing.Any:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        def __xor__(self, other: typing.Any) -> typing.Any:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    cacheLinearizedFactors: bool
    enableDetailedResults: bool
    enablePartialRelinearizationCheck: bool
    enableRelinearization: bool
    evaluateNonlinearError: bool
    factorization: ...
    findUnusedFactorSlots: bool
    relinearizeSkip: int
    def __init__(self) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def getFactorization(self) -> str:
        ...
    def print(self, str: str = '') -> None:
        """
        print iSAM2 parameters
        """
    def setFactorization(self, factorization: str) -> None:
        ...
    @typing.overload
    def setOptimizationParams(self, gauss_newton__params: ISAM2GaussNewtonParams) -> None:
        ...
    @typing.overload
    def setOptimizationParams(self, optimizationParams: ISAM2DoglegParams) -> None:
        ...
    @typing.overload
    def setRelinearizeThreshold(self, relinearizeThreshold: float) -> None:
        ...
    @typing.overload
    def setRelinearizeThreshold(self, threshold_map: ISAM2ThresholdMap) -> None:
        ...
class ISAM2Result:
    def __init__(self) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def getCliques(self) -> int:
        ...
    def getErrorAfter(self) -> float:
        ...
    def getErrorBefore(self) -> float:
        ...
    def getNewFactorsIndices(self) -> list[int]:
        ...
    def getVariablesReeliminated(self) -> int:
        ...
    def getVariablesRelinearized(self) -> int:
        """
        Getters and Setters.
        """
    def print(self, str: str = '') -> None:
        """
        Print results.
        """
class ISAM2ThresholdMap:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: ISAM2ThresholdMap) -> None:
        ...
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def insert(self, value: tuple[str, numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]) -> None:
        ...
    def size(self) -> int:
        ...
class ImuFactor(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, pose_i: int, vel_i: int, pose_j: int, vel_j: int, bias: int, preintegratedMeasurements: PreintegratedImuMeasurements) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def evaluateError(self, pose_i: Pose3, vel_i: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], pose_j: Pose3, vel_j: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], bias: imuBias.ConstantBias) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def preintegratedMeasurements(self) -> PreintegratedImuMeasurements:
        """
        Access the preintegrated measurements.
        """
    def serialize(self) -> str:
        ...
class ImuFactor2(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, state_i: int, state_j: int, bias: int, preintegratedMeasurements: PreintegratedImuMeasurements) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def evaluateError(self, state_i: NavState, state_j: NavState, bias_i: imuBias.ConstantBias) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def preintegratedMeasurements(self) -> PreintegratedImuMeasurements:
        """
        Access the preintegrated measurements.
        """
    def serialize(self) -> str:
        ...
class IncrementalFixedLagSmoother(FixedLagSmoother):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, smootherLag: float) -> None:
        ...
    @typing.overload
    def __init__(self, smootherLag: float, parameters: ISAM2Params) -> None:
        ...
    def __repr__(self, s: str = 'IncrementalFixedLagSmoother:\n') -> str:
        ...
    def getFactors(self) -> NonlinearFactorGraph:
        """
        Access the current set of factors.
        """
    def getISAM2(self) -> ISAM2:
        """
        Get the iSAM2 object which is used for the inference internally.
        """
    def marginalCovariance(self, key: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Calculate marginal covariance on given variable.
        """
    def params(self) -> ISAM2Params:
        """
        return the current set of iSAM2 parameters
        """
    def print(self, s: str = 'IncrementalFixedLagSmoother:\n') -> None:
        ...
class IndexPair:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, i: int, j: int) -> None:
        ...
    def i(self) -> int:
        ...
    def j(self) -> int:
        ...
class InitializePose3:
    @staticmethod
    def buildPose3graph(graph: NonlinearFactorGraph) -> NonlinearFactorGraph:
        """
        Select the subgraph of betweenFactors and transforms priors into between wrt a fictitious node.
        """
    @staticmethod
    def computeOrientationsChordal(pose3Graph: NonlinearFactorGraph) -> Values:
        """
        Return the orientations of a graph including only BetweenFactors<Pose3>
        """
    @staticmethod
    @typing.overload
    def computeOrientationsGradient(pose3Graph: NonlinearFactorGraph, givenGuess: Values, maxIter: int, setRefFrame: bool) -> Values:
        """
        Return the orientations of a graph including only BetweenFactors<Pose3>
        """
    @staticmethod
    @typing.overload
    def computeOrientationsGradient(pose3Graph: NonlinearFactorGraph, givenGuess: Values) -> Values:
        """
        Return the orientations of a graph including only BetweenFactors<Pose3>
        """
    @staticmethod
    @typing.overload
    def initialize(graph: NonlinearFactorGraph, givenGuess: Values, useGradient: bool) -> Values:
        """
        "extract" the Pose3 subgraph of the original graph, get orientations from relative orientation measurements (using either gradient or chordal method), and finish up with 1 GN iteration on full poses.
        """
    @staticmethod
    @typing.overload
    def initialize(graph: NonlinearFactorGraph) -> Values:
        """
        Calls initialize above using Chordal method.
        """
    @staticmethod
    def initializeOrientations(graph: NonlinearFactorGraph) -> Values:
        """
        "extract" the Pose3 subgraph of the original graph, get orientations from relative orientation measurements using chordal method.
        """
class IterativeOptimizationParameters:
    def getVerbosity(self) -> str:
        ...
    def setVerbosity(self, s: str) -> None:
        ...
class JacobianFactor(GaussianFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, b_in: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, i1: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, i1: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], i2: int, A2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, i1: int, A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], i2: int, A2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], i3: int, A3: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self, graph: ...) -> None:
        ...
    @typing.overload
    def __init__(self, graph: ..., p_variableSlots: ...) -> None:
        ...
    @typing.overload
    def __init__(self, graph: ..., ordering: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, graph: ..., ordering: Ordering, p_variableSlots: ...) -> None:
        ...
    @typing.overload
    def __init__(self, factor: GaussianFactor) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def augmentedJacobianUnweighted(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return (dense) matrix associated with factor. 
        The returned system is an augmented matrix: [A b] weights are not baked in
        """
    def cols(self) -> int:
        """
        return the number of columns in the corresponding linear system
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def eliminate(self, keys: Ordering) -> tuple[..., JacobianFactor]:
        """
        Eliminate the requested variables.
        """
    def equals(self, lf: GaussianFactor, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def error(self, c: VectorValues) -> float:
        ...
    def error_vector(self, c: VectorValues) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        (A*x-b)
        """
    def getA(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Get a view of the A matrix, not weighted by noise.
        """
    def get_model(self) -> noiseModel.Diagonal:
        """
        get a copy of model
        """
    def getb(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get a view of the r.h.s. 
        vector b, not weighted by noise
        """
    def isConstrained(self) -> bool:
        """
        is noise model constrained ?
        """
    def jacobianUnweighted(self) -> tuple[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Returns (dense) A,b pair associated with factor, does not bake in weights.
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def rows(self) -> int:
        """
        return the number of rows in the corresponding linear system
        """
    def serialize(self) -> str:
        ...
    def setModel(self, anyConstrained: bool, sigmas: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        """
        set noiseModel correctly
        """
    def transposeMultiplyAdd(self, alpha: float, e: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], x: VectorValues) -> None:
        """
        x += alpha * A'*e. 
        If x is initially missing any values, they are created and assumed to start as zero vectors.
        """
    def unweighted_error(self, c: VectorValues) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def whiten(self) -> JacobianFactor:
        """
        Return a whitened version of the factor, i.e. 
        with unit diagonal noise model.
        """
class JacobianVector:
    __hash__: typing.ClassVar[None] = None
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    def __contains__(self, x: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> bool:
        """
        Return true the container contains ``x``
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    def __eq__(self, arg0: JacobianVector) -> bool:
        ...
    @typing.overload
    def __getitem__(self, s: slice) -> JacobianVector:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: JacobianVector) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None:
        ...
    def __iter__(self) -> typing.Iterator[numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]]:
        ...
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: JacobianVector) -> bool:
        ...
    def __repr__(self) -> str:
        """
        Return the canonical string representation of this list.
        """
    @typing.overload
    def __setitem__(self, arg0: int, arg1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: JacobianVector) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    def count(self, x: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> int:
        """
        Return the number of times ``x`` appears in the list
        """
    @typing.overload
    def extend(self, L: JacobianVector) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: int, x: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Remove and return the item at index ``i``
        """
    def remove(self, x: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        """
        Remove the first item from the list whose value is x. It is an error if there is no such item.
        """
class JointMarginal:
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, iVariable: int, jVariable: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Synonym for operator()
        """
    def fullMatrix(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        The full, dense covariance/information matrix of the joint marginal.
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class KalmanFilter:
    @staticmethod
    def step(p: GaussianDensity) -> int:
        """
        Return the step index $ k $ (starts at 0, incremented at each predict step). 
        p: The current state.
        Returns: Step index.
        """
    def __init__(self, n: int) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def init(self, x0: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], P0: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> GaussianDensity:
        """
        Create the initial state (prior density at time $ k=0 $). 
        In Kalman Filter notation:$ x_{0|0} $: Initial state estimate.$ P_{0|0} $: Initial covariance matrix. x0: Estimate of the state at time 0 (
        P0: Covariance matrix (
        Returns: Initial Kalman filter state.
        """
    def predict(self, p: GaussianDensity, F: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], B: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], u: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], modelQ: noiseModel.Diagonal) -> GaussianDensity:
        """
        Predict the next state $ P(x_{k+1}|Z^k) $. 
        In Kalman Filter notation:$ x_{k+1|k} $: Predicted state.$ P_{k+1|k} $: Predicted covariance. Motion model: \\[ x_{k+1} = F \\cdot x_k + B \\cdot u_k + w \\] where $ w $ is zero-mean Gaussian noise with covariance $ Q $. p: Previous state (
        F: State transition matrix (
        B: Control input matrix (
        u: Control vector (
        modelQ: Noise model (
        Returns: Predicted state (
        """
    def predict2(self, p: GaussianDensity, A0: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], A1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> GaussianDensity:
        """
        Predict the next state using a GaussianFactor motion model. 
        p: Previous state.
        A0: No description provided
        A1: No description provided
        b: Constant term vector.
        model: Noise model (optional).
        Returns: Predicted state.
        """
    def predictQ(self, p: GaussianDensity, F: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], B: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], u: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], Q: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> GaussianDensity:
        """
        Predict the next state with a full covariance matrix. 
        p: Previous state.
        F: State transition matrix.
        B: Control input matrix.
        u: Control vector.
        Q: Full covariance matrix (
        """
    def print(self, s: str = '') -> None:
        """
        Print the Kalman filter details. 
        s: Optional string prefix.
        """
    def update(self, p: GaussianDensity, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Diagonal) -> GaussianDensity:
        """
        Update the Kalman filter with a measurement. 
        Observation model: \\[ z_k = H \\cdot x_k + v \\] where $ v $ is zero-mean Gaussian noise with covariance R. In this version, R is restricted to diagonal Gaussians (model parameter) p: Previous state.
        H: Observation matrix.
        z: Measurement vector.
        model: Noise model (diagonal Gaussian).
        Returns: Updated state.
        """
    def updateQ(self, p: GaussianDensity, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], Q: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> GaussianDensity:
        ...
class KarcherMeanFactorPoint2(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorPoint3(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorPose2(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorPose3(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorRot2(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorRot3(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorSO3(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KarcherMeanFactorSO4(NonlinearFactor):
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int], d: int, beta: float) -> None:
        ...
class KernelFunctionType:
    """
    Members:
    
      KernelFunctionTypeNONE
    
      KernelFunctionTypeHUBER
    
      KernelFunctionTypeTUKEY
    """
    KernelFunctionTypeHUBER: typing.ClassVar[KernelFunctionType]  # value = <KernelFunctionType.KernelFunctionTypeHUBER: 1>
    KernelFunctionTypeNONE: typing.ClassVar[KernelFunctionType]  # value = <KernelFunctionType.KernelFunctionTypeNONE: 0>
    KernelFunctionTypeTUKEY: typing.ClassVar[KernelFunctionType]  # value = <KernelFunctionType.KernelFunctionTypeTUKEY: 2>
    __members__: typing.ClassVar[dict[str, KernelFunctionType]]  # value = {'KernelFunctionTypeNONE': <KernelFunctionType.KernelFunctionTypeNONE: 0>, 'KernelFunctionTypeHUBER': <KernelFunctionType.KernelFunctionTypeHUBER: 1>, 'KernelFunctionTypeTUKEY': <KernelFunctionType.KernelFunctionTypeTUKEY: 2>}
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class KeyGroupMap:
    def __init__(self) -> None:
        ...
    def at(self, key: int) -> int:
        ...
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def erase(self, key: int) -> int:
        ...
    def insert2(self, key: int, val: int) -> bool:
        ...
    def size(self) -> int:
        ...
class KeyList:
    def __contains__(self, key: int) -> bool:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: KeyList) -> None:
        ...
    def __iter__(self) -> typing.Iterator[int]:
        ...
    def __len__(self) -> int:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def back(self) -> int:
        ...
    def clear(self) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def empty(self) -> bool:
        ...
    def front(self) -> int:
        ...
    def pop_back(self) -> None:
        ...
    def pop_front(self) -> None:
        ...
    def push_back(self, key: int) -> None:
        ...
    def push_front(self, key: int) -> None:
        ...
    def remove(self, key: int) -> None:
        ...
    def serialize(self) -> str:
        ...
    def size(self) -> int:
        ...
    def sort(self) -> None:
        ...
class KeySet:
    def __contains__(self, key: int) -> bool:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, set: KeySet) -> None:
        ...
    @typing.overload
    def __init__(self, vector: list[int]) -> None:
        ...
    @typing.overload
    def __init__(self, list: KeyList) -> None:
        ...
    def __iter__(self) -> typing.Iterator[int]:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def clear(self) -> None:
        ...
    def count(self, key: int) -> int:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, other: KeySet) -> bool:
        ...
    def erase(self, key: int) -> int:
        ...
    def insert(self, key: int) -> None:
        ...
    def merge(self, other: KeySet) -> None:
        ...
    def print(self, s: str = '') -> None:
        ...
    def serialize(self) -> str:
        ...
    def size(self) -> int:
        ...
class LabeledSymbol:
    @typing.overload
    def __init__(self, full_key: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: LabeledSymbol) -> None:
        ...
    @typing.overload
    def __init__(self, valType: int, label: int, j: int) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def chr(self) -> int:
        """
        Retrieve key character.
        """
    def index(self) -> int:
        """
        Retrieve key index.
        """
    def key(self) -> int:
        """
        return the integer version
        """
    def label(self) -> int:
        """
        Retrieve label character.
        """
    def lower(self) -> LabeledSymbol:
        ...
    def newChr(self, c: int) -> LabeledSymbol:
        """
        Create a new symbol with a different character.
        """
    def newLabel(self, label: int) -> LabeledSymbol:
        """
        Create a new symbol with a different label.
        """
    def print(self, s: str = '') -> None:
        """
        Prints the LabeledSymbol with an optional prefix string.
        """
    def upper(self) -> LabeledSymbol:
        """
        Converts to upper/lower versions of labels.
        """
class LevenbergMarquardtOptimizer(NonlinearOptimizer):
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ..., params: LevenbergMarquardtParams = ...) -> None:
        ...
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, initialValues: ..., ordering: Ordering, params: LevenbergMarquardtParams = ...) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def lambda_(self) -> float:
        """
        Access the current damping value.
        """
    def print(self, str: str = '') -> None:
        """
        print
        """
class LevenbergMarquardtParams(NonlinearOptimizerParams):
    @staticmethod
    def CeresDefaults() -> LevenbergMarquardtParams:
        ...
    @staticmethod
    def EnsureHasOrdering(params: LevenbergMarquardtParams, graph: NonlinearFactorGraph) -> LevenbergMarquardtParams:
        ...
    @staticmethod
    def LegacyDefaults() -> LevenbergMarquardtParams:
        ...
    @staticmethod
    def ReplaceOrdering(params: LevenbergMarquardtParams, ordering: Ordering) -> LevenbergMarquardtParams:
        ...
    def __init__(self) -> None:
        ...
    def getDiagonalDamping(self) -> bool:
        ...
    def getLogFile(self) -> str:
        ...
    def getUseFixedLambdaFactor(self) -> bool:
        ...
    def getVerbosityLM(self) -> str:
        ...
    def getlambdaFactor(self) -> float:
        ...
    def getlambdaInitial(self) -> float:
        ...
    def getlambdaLowerBound(self) -> float:
        ...
    def getlambdaUpperBound(self) -> float:
        ...
    def setDiagonalDamping(self, flag: bool) -> None:
        ...
    def setLogFile(self, s: str) -> None:
        ...
    def setUseFixedLambdaFactor(self, flag: bool) -> None:
        ...
    def setVerbosityLM(self, s: str) -> None:
        ...
    def setlambdaFactor(self, value: float) -> None:
        ...
    def setlambdaInitial(self, value: float) -> None:
        ...
    def setlambdaLowerBound(self, value: float) -> None:
        ...
    def setlambdaUpperBound(self, value: float) -> None:
        ...
class LinearContainerFactor(NonlinearFactor):
    @staticmethod
    @typing.overload
    def ConvertLinearGraph(linear_graph: GaussianFactorGraph, linearizationPoint: ...) -> NonlinearFactorGraph:
        """
        Utility function for converting linear graphs to nonlinear graphs consisting of LinearContainerFactors.
        """
    @staticmethod
    @typing.overload
    def ConvertLinearGraph(linear_graph: GaussianFactorGraph) -> NonlinearFactorGraph:
        """
        Utility function for converting linear graphs to nonlinear graphs consisting of LinearContainerFactors.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, factor: GaussianFactor, linearizationPoint: ...) -> None:
        ...
    @typing.overload
    def __init__(self, factor: GaussianFactor) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def factor(self) -> GaussianFactor:
        ...
    def isJacobian(self) -> bool:
        """
        Simple checks whether this is a Jacobian or Hessian factor.
        """
    def serialize(self) -> str:
        ...
    def toHessian(self) -> HessianFactor:
        """
        Casts to HessianFactor.
        """
    def toJacobian(self) -> JacobianFactor:
        """
        Casts to JacobianFactor.
        """
class LinearizationMode:
    """
    Members:
    
      HESSIAN
    
      IMPLICIT_SCHUR
    
      JACOBIAN_Q
    
      JACOBIAN_SVD
    """
    HESSIAN: typing.ClassVar[LinearizationMode]  # value = <LinearizationMode.HESSIAN: 0>
    IMPLICIT_SCHUR: typing.ClassVar[LinearizationMode]  # value = <LinearizationMode.IMPLICIT_SCHUR: 1>
    JACOBIAN_Q: typing.ClassVar[LinearizationMode]  # value = <LinearizationMode.JACOBIAN_Q: 2>
    JACOBIAN_SVD: typing.ClassVar[LinearizationMode]  # value = <LinearizationMode.JACOBIAN_SVD: 3>
    __members__: typing.ClassVar[dict[str, LinearizationMode]]  # value = {'HESSIAN': <LinearizationMode.HESSIAN: 0>, 'IMPLICIT_SCHUR': <LinearizationMode.IMPLICIT_SCHUR: 1>, 'JACOBIAN_Q': <LinearizationMode.JACOBIAN_Q: 2>, 'JACOBIAN_SVD': <LinearizationMode.JACOBIAN_SVD: 3>}
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MFAS:
    def __init__(self, relativeTranslations: list[...], projectionDirection: Unit3) -> None:
        ...
    def computeOrdering(self) -> list[int]:
        """
        Computes the 1D MFAS ordering of nodes in the graph. 
        orderedNodes: vector of nodes in the obtained order  Returns: orderedNodes: vector of nodes in the obtained order
        """
    def computeOutlierWeights(self) -> dict[tuple[int, int], float]:
        """
        Computes the outlier weights of the graph. 
        We define the outlier weight of a edge to be zero if the edge is an inlier and the magnitude of its edgeWeight if it is an outlier. This function internally calls computeOrdering and uses the obtained ordering to identify outlier edges. outlierWeights: map from an edge to its outlier weight.  Returns: outlierWeights: map from an edge to its outlier weight.
        """
class MT19937:
    def __call__(self) -> int:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: int) -> None:
        ...
class MagFactor(NonlinearFactor):
    def __init__(self, key: int, measured: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], scale: float, direction: Unit3, bias: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, nRb: Rot2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class MagFactor1(NonlinearFactor):
    def __init__(self, key: int, measured: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], scale: float, direction: Unit3, bias: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, nRb: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class MagPoseFactorPose2(NoiseModelFactor):
    @typing.overload
    def __init__(self, pose_key: int, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], scale: float, direction: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bias: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    @typing.overload
    def __init__(self, pose_key: int, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], scale: float, direction: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bias: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, body_P_sensor: Pose2) -> None:
        ...
    def evaluateError(self, nRb: Pose2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class MagPoseFactorPose3(NoiseModelFactor):
    @typing.overload
    def __init__(self, pose_key: int, measured: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], scale: float, direction: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], bias: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    @typing.overload
    def __init__(self, pose_key: int, measured: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], scale: float, direction: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], bias: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base, body_P_sensor: Pose3) -> None:
        ...
    def evaluateError(self, nRb: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class ManifoldEvaluationFactorChebyshev1BasisPose2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev1BasisPose3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev1BasisRot2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev1BasisRot3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2BasisPose2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2BasisPose3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2BasisRot2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2BasisRot3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2Pose2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2Pose3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2Rot2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorChebyshev2Rot3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorFourierBasisPose2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorFourierBasisPose3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Pose3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorFourierBasisRot2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot2, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class ManifoldEvaluationFactorFourierBasisRot3(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: Rot3, model: noiseModel.Base, N: int, x: float, a: float, b: float) -> None:
        ...
class Marginals:
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph, solution: ...) -> None:
        ...
    @typing.overload
    def __init__(self, gfgraph: GaussianFactorGraph, solution: ...) -> None:
        ...
    @typing.overload
    def __init__(self, gfgraph: GaussianFactorGraph, solutionvec: VectorValues) -> None:
        ...
    def __repr__(self, s: str = 'Marginals: ', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def jointMarginalCovariance(self, variables: list[int]) -> ...:
        """
        Compute the joint marginal covariance of several variables.
        """
    def jointMarginalInformation(self, variables: list[int]) -> ...:
        """
        Compute the joint marginal information of several variables.
        """
    def marginalCovariance(self, variable: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Compute the marginal covariance of a single variable.
        """
    def marginalInformation(self, variable: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Compute the marginal information matrix of a single variable. 
        Use LLt(const Matrix&) or RtR(const Matrix&) to obtain the square-root information matrix.
        """
    def print(self, s: str = 'Marginals: ', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class NavState:
    @staticmethod
    @typing.overload
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> NavState:
        ...
    @staticmethod
    @typing.overload
    def Expmap(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], Hxi: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> NavState:
        """
        Exponential map at identity - create a NavState from canonical coordinates $ [R_x,R_y,R_z,T_x,T_y,T_z,V_x,V_y,V_z] $.
        """
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[5], typing.Literal[5]], numpy.dtype[numpy.float64]]:
        """
        Hat maps from tangent vector to Lie algebra.
        """
    @staticmethod
    @typing.overload
    def Logmap(p: NavState) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    @typing.overload
    def Logmap(pose: NavState, Hpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at identity - return the canonical coordinates $ [R_x,R_y,R_z,T_x,T_y,T_z,V_x,V_y,V_z] $ of this NavState.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    def Adjoint(self, xi_b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Apply this NavState's AdjointMap Ad_g to a twist $ \\xi_b $, i.e. 
        a body-fixed velocity, transforming it to the spatial frame $ \\xi^s = g*\\xi^b*g^{-1} = Ad_g * \\xi^b $ Note that H_xib = AdjointMap()
        """
    def AdjointMap(self) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[9]], numpy.dtype[numpy.float64]]:
        """
        Calculate Adjoint map, transforming a twist in this pose's (i.e, body) frame to the world spatial frame.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, R: Rot3, t: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def attitude(self) -> Rot3:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: NavState, tol: float) -> bool:
        """
        equals
        """
    @typing.overload
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> NavState:
        ...
    @typing.overload
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> NavState:
        ...
    def localCoordinates(self, g: NavState) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        localCoordinates with optional derivatives
        """
    @typing.overload
    def logmap(self, p: NavState) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def logmap(self, p: NavState, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def position(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        """
        print
        """
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> NavState:
        """
        retract with optional derivatives
        """
    def serialize(self) -> str:
        ...
    def velocity(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NoiseFormat:
    """
    Members:
    
      NoiseFormatG2O
    
      NoiseFormatTORO
    
      NoiseFormatGRAPH
    
      NoiseFormatCOV
    
      NoiseFormatAUTO
    """
    NoiseFormatAUTO: typing.ClassVar[NoiseFormat]  # value = <NoiseFormat.NoiseFormatAUTO: 4>
    NoiseFormatCOV: typing.ClassVar[NoiseFormat]  # value = <NoiseFormat.NoiseFormatCOV: 3>
    NoiseFormatG2O: typing.ClassVar[NoiseFormat]  # value = <NoiseFormat.NoiseFormatG2O: 0>
    NoiseFormatGRAPH: typing.ClassVar[NoiseFormat]  # value = <NoiseFormat.NoiseFormatGRAPH: 2>
    NoiseFormatTORO: typing.ClassVar[NoiseFormat]  # value = <NoiseFormat.NoiseFormatTORO: 1>
    __members__: typing.ClassVar[dict[str, NoiseFormat]]  # value = {'NoiseFormatG2O': <NoiseFormat.NoiseFormatG2O: 0>, 'NoiseFormatTORO': <NoiseFormat.NoiseFormatTORO: 1>, 'NoiseFormatGRAPH': <NoiseFormat.NoiseFormatGRAPH: 2>, 'NoiseFormatCOV': <NoiseFormat.NoiseFormatCOV: 3>, 'NoiseFormatAUTO': <NoiseFormat.NoiseFormatAUTO: 4>}
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class NoiseModelFactor(NonlinearFactor):
    def cloneWithNewNoiseModel(self, newNoise: noiseModel.Base) -> NoiseModelFactor:
        """
        Creates a shared_ptr clone of the factor with a new noise model.
        """
    def equals(self, f: NoiseModelFactor, tol: float) -> bool:
        """
        Check if two factors are equal.
        """
    def noiseModel(self) -> noiseModel.Base:
        """
        access to the noise model
        """
    def unwhitenedError(self, x: ...) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Error function without the NoiseModel, $ z-h(x) $. 
        Override this method to finish implementing an N-way factor. If the optional arguments is specified, it should compute both the function evaluation and its derivative(s) in H.
        """
    def whitenedError(self, c: ...) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vector of errors, whitened This is the raw error, i.e., i.e. 
        $ (h(x)-z)/\\sigma $ in case of a Gaussian
        """
class NonlinearEquality2Cal3_S2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Cal3_S2, x2: Cal3_S2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2CalibratedCamera(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: CalibratedCamera, x2: CalibratedCamera) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2ConstantBias(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: ..., x2: ...) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2PinholeCameraCal3Bundler(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: PinholeCameraCal3Bundler, x2: PinholeCameraCal3Bundler) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2PinholeCameraCal3Fisheye(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: PinholeCameraCal3Fisheye, x2: PinholeCameraCal3Fisheye) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2PinholeCameraCal3Unified(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: PinholeCameraCal3Unified, x2: PinholeCameraCal3Unified) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2PinholeCameraCal3_S2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: PinholeCameraCal3_S2, x2: PinholeCameraCal3_S2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Point2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], x2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Point3(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], x2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Pose2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Pose2, x2: Pose2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Pose3(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Pose3, x2: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Rot2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Rot2, x2: Rot2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Rot3(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Rot3, x2: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2SO3(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: SO3, x2: SO3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2SO4(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: SO4, x2: SO4) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2SOn(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: SOn, x2: SOn) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Similarity2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Similarity2, x2: Similarity2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2Similarity3(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: Similarity3, x2: Similarity3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEquality2StereoPoint2(NoiseModelFactor):
    def __init__(self, key1: int, key2: int, mu: float = 10000.0) -> None:
        ...
    def evaluateError(self, x1: StereoPoint2, x2: StereoPoint2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class NonlinearEqualityCal3_S2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Cal3_S2) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Cal3_S2, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityCalibratedCamera(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: CalibratedCamera) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: CalibratedCamera, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityConstantBias(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: ...) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: ..., error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPinholeCameraCal3Bundler(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3Bundler) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3Bundler, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPinholeCameraCal3Fisheye(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3Fisheye) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3Fisheye, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPinholeCameraCal3Unified(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3Unified) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3Unified, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPinholeCameraCal3_S2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3_S2) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: PinholeCameraCal3_S2, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPoint2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPoint3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Pose2) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Pose2, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityPose3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Pose3, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityRot2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Rot2) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Rot2, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityRot3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Rot3) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Rot3, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualitySO3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: SO3) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: SO3, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualitySO4(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: SO4) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: SO4, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualitySOn(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: SOn) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: SOn, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualitySimilarity2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Similarity2) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Similarity2, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualitySimilarity3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Similarity3) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: Similarity3, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearEqualityStereoPoint2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: StereoPoint2) -> None:
        ...
    @typing.overload
    def __init__(self, j: int, feasible: StereoPoint2, error_gain: float) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class NonlinearFactor(Factor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def active(self, c: ...) -> bool:
        """
        Checks whether a factor should be used based on a set of values. 
        This is primarily used to implement inequality constraints that require a variable active set. For all others, the default implementation returning true solves this problem. In an inequality/bounding constraint, this active() returns true when the constraint is NOT fulfilled. true if the constraint is active  Returns: true if the constraint is active
        """
    def clone(self) -> NonlinearFactor:
        """
        Creates a shared_ptr clone of the factor - needs to be specialized to allow for subclasses. 
        By default, throws exception if subclass does not implement the function.
        """
    def dim(self) -> int:
        """
        get the dimension of the factor (number of rows on linearization)
        """
    def equals(self, f: NonlinearFactor, tol: float) -> bool:
        """
        Check if two factors are equal.
        """
    @typing.overload
    def error(self, c: ...) -> float:
        ...
    @typing.overload
    def error(self, c: ...) -> float:
        """
        All factor types need to implement an error function. 
        In factor graphs, this is the negative log-likelihood.
        """
    def linearize(self, c: ...) -> GaussianFactor:
        """
        linearize to a GaussianFactor
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def rekey(self, newKeys: list[int]) -> NonlinearFactor:
        ...
class NonlinearFactorGraph:
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, graph: NonlinearFactorGraph) -> None:
        ...
    def __repr__(self, s: str = 'NonlinearFactorGraph: ', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add(self, factor: ...) -> None:
        ...
    def addPriorCal3Bundler(self, key: int, prior: Cal3Bundler, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorCal3Fisheye(self, key: int, prior: Cal3Fisheye, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorCal3Unified(self, key: int, prior: Cal3Unified, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorCal3_S2(self, key: int, prior: Cal3_S2, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorCal3f(self, key: int, prior: Cal3f, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorCalibratedCamera(self, key: int, prior: CalibratedCamera, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorConstantBias(self, key: int, prior: ..., noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorDouble(self, key: int, prior: float, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorEssentialMatrix(self, key: int, prior: EssentialMatrix, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorFundamentalMatrix(self, key: int, prior: FundamentalMatrix, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPinholeCameraCal3Bundler(self, key: int, prior: PinholeCameraCal3Bundler, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPinholeCameraCal3Fisheye(self, key: int, prior: PinholeCameraCal3Fisheye, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPinholeCameraCal3Unified(self, key: int, prior: PinholeCameraCal3Unified, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPinholeCameraCal3_S2(self, key: int, prior: PinholeCameraCal3_S2, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPinholeCameraCal3f(self, key: int, prior: PinholeCameraCal3f, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPinholeCameraCalibratedCamera(self, key: int, prior: ..., noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPoint2(self, key: int, prior: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPoint3(self, key: int, prior: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPose2(self, key: int, prior: Pose2, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorPose3(self, key: int, prior: Pose3, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorRot2(self, key: int, prior: Rot2, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorRot3(self, key: int, prior: Rot3, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorSO3(self, key: int, prior: SO3, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorSO4(self, key: int, prior: SO4, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorSimilarity2(self, key: int, prior: Similarity2, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorSimilarity3(self, key: int, prior: Similarity3, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorSimpleFundamentalMatrix(self, key: int, prior: SimpleFundamentalMatrix, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorStereoPoint2(self, key: int, prior: StereoPoint2, noiseModel: noiseModel.Base) -> None:
        ...
    def addPriorVector(self, key: int, prior: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def at(self, idx: int) -> ...:
        ...
    def clone(self) -> NonlinearFactorGraph:
        """
        Clone() performs a deep-copy of the graph, including all of the factors.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dot(self, values: ..., keyFormatter: typing.Callable[[int], str] = ..., writer: GraphvizFormatting = ...) -> str:
        """
        Output to graphviz format string, with Values/extra options.
        """
    def empty(self) -> bool:
        ...
    def equals(self, other: NonlinearFactorGraph, tol: float) -> bool:
        """
        Test equality.
        """
    def error(self, values: ...) -> float:
        """
        unnormalized error, $ \\sum_i 0.5 (h_i(X_i)-z)^2 / \\sigma^2 $ in the most common case
        """
    def exists(self, idx: int) -> bool:
        ...
    def keyVector(self) -> list[int]:
        ...
    def keys(self) -> ...:
        ...
    def linearize(self, linearizationPoint: ...) -> GaussianFactorGraph:
        """
        Linearize a nonlinear factor graph.
        """
    def nrFactors(self) -> int:
        ...
    def orderingCOLAMD(self) -> Ordering:
        """
        Compute a fill-reducing ordering using COLAMD.
        """
    def print(self, s: str = 'NonlinearFactorGraph: ', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def printErrors(self, values: ..., str: str = 'NonlinearFactorGraph: ', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def probPrime(self, values: ...) -> float:
        """
        Unnormalized probability. 
        O(n)
        """
    @typing.overload
    def push_back(self, factors: NonlinearFactorGraph) -> None:
        ...
    @typing.overload
    def push_back(self, factor: ...) -> None:
        ...
    def remove(self, i: int) -> None:
        ...
    def replace(self, i: int, factors: ...) -> None:
        ...
    def resize(self, size: int) -> None:
        ...
    def saveGraph(self, s: str, values: ..., keyFormatter: typing.Callable[[int], str] = ..., writer: GraphvizFormatting = ...) -> None:
        ...
    def serialize(self) -> str:
        ...
    def size(self) -> int:
        ...
class NonlinearISAM:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, reorderInterval: int) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def bayesTree(self) -> GaussianISAM:
        """
        access the underlying bayes tree
        """
    def estimate(self) -> ...:
        """
        Return the current solution estimate.
        """
    def getFactorsUnsafe(self) -> NonlinearFactorGraph:
        """
        get underlying nonlinear graph
        """
    def getLinearizationPoint(self) -> ...:
        """
        Return the current linearization point.
        """
    def marginalCovariance(self, key: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        find the marginal covariance for a single variable
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        prints out all contents of the system
        """
    def printStats(self) -> None:
        """
        prints out clique statistics
        """
    def reorderCounter(self) -> int:
        """
        TODO: comment.
        """
    def reorderInterval(self) -> int:
        """
        get counters 
        TODO: comment
        """
    def reorder_relinearize(self) -> None:
        """
        Relinearization and reordering of variables.
        """
    def saveGraph(self, s: str) -> None:
        """
        saves the Tree to a text file in GraphViz format
        """
    def update(self, newFactors: NonlinearFactorGraph, initialValues: ...) -> None:
        """
        Add new factors along with their initial linearization points.
        """
class NonlinearOptimizer:
    def error(self) -> float:
        """
        return error in current optimizer state
        """
    def graph(self) -> NonlinearFactorGraph:
        """
        return the graph with nonlinear factors
        """
    def iterate(self) -> GaussianFactorGraph:
        """
        Perform a single iteration, returning GaussianFactorGraph corresponding to the linearized factor graph.
        """
    def iterations(self) -> int:
        """
        return number of iterations in current optimizer state
        """
    def optimize(self) -> ...:
        """
        Optimize for the maximum-likelihood estimate, returning a the optimized variable assignments. 
        This function simply calls iterate() in a loop, checking for convergence with check_convergence(). For fine-grain control over the optimization process, you may call iterate() and check_convergence() yourself, and if needed modify the optimization state between iterations.
        """
    def optimizeSafely(self) -> ...:
        """
        Optimize, but return empty result if any uncaught exception is thrown Intended for MATLAB. 
        optimizer: a non-linear optimizer
        """
    def values(self) -> ...:
        """
        return values in current optimizer state
        """
class NonlinearOptimizerParams:
    iterationHook: typing.Callable[[int, float, float], None]
    def __init__(self) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def getAbsoluteErrorTol(self) -> float:
        ...
    def getErrorTol(self) -> float:
        ...
    def getLinearSolverType(self) -> str:
        ...
    def getMaxIterations(self) -> int:
        ...
    def getOrderingType(self) -> str:
        ...
    def getRelativeErrorTol(self) -> float:
        ...
    def getVerbosity(self) -> str:
        ...
    def isCholmod(self) -> bool:
        ...
    def isIterative(self) -> bool:
        ...
    def isMultifrontal(self) -> bool:
        ...
    def isSequential(self) -> bool:
        ...
    def print(self, str: str = '') -> None:
        ...
    def setAbsoluteErrorTol(self, value: float) -> None:
        ...
    def setErrorTol(self, value: float) -> None:
        ...
    def setIterativeParams(self, params: IterativeOptimizationParameters) -> None:
        ...
    def setLinearSolverType(self, solver: str) -> None:
        ...
    def setMaxIterations(self, value: int) -> None:
        ...
    def setOrdering(self, ordering: Ordering) -> None:
        ...
    def setOrderingType(self, ordering: str) -> None:
        ...
    def setRelativeErrorTol(self, value: float) -> None:
        ...
    def setVerbosity(self, src: str) -> None:
        ...
class Ordering:
    class OrderingType:
        """
        Members:
        
          COLAMD
        
          METIS
        
          NATURAL
        
          CUSTOM
        """
        COLAMD: typing.ClassVar[Ordering.OrderingType]  # value = <OrderingType.COLAMD: 0>
        CUSTOM: typing.ClassVar[Ordering.OrderingType]  # value = <OrderingType.CUSTOM: 3>
        METIS: typing.ClassVar[Ordering.OrderingType]  # value = <OrderingType.METIS: 1>
        NATURAL: typing.ClassVar[Ordering.OrderingType]  # value = <OrderingType.NATURAL: 2>
        __members__: typing.ClassVar[dict[str, Ordering.OrderingType]]  # value = {'COLAMD': <OrderingType.COLAMD: 0>, 'METIS': <OrderingType.METIS: 1>, 'NATURAL': <OrderingType.NATURAL: 2>, 'CUSTOM': <OrderingType.CUSTOM: 3>}
        def __and__(self, other: typing.Any) -> typing.Any:
            ...
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __ge__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __gt__(self, other: typing.Any) -> bool:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __invert__(self) -> typing.Any:
            ...
        def __le__(self, other: typing.Any) -> bool:
            ...
        def __lt__(self, other: typing.Any) -> bool:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __or__(self, other: typing.Any) -> typing.Any:
            ...
        def __rand__(self, other: typing.Any) -> typing.Any:
            ...
        def __repr__(self) -> str:
            ...
        def __ror__(self, other: typing.Any) -> typing.Any:
            ...
        def __rxor__(self, other: typing.Any) -> typing.Any:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        def __xor__(self, other: typing.Any) -> typing.Any:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    @staticmethod
    def Colamd(variableIndex: ...) -> Ordering:
        """
        Compute a fill-reducing ordering using COLAMD from a VariableIndex.
        """
    @staticmethod
    def ColamdConstrainedFirstDiscreteFactorGraph(graph: ..., constrainFirst: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedFirstGaussianFactorGraph(graph: ..., constrainFirst: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedFirstHybridGaussianFactorGraph(graph: ..., constrainFirst: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedFirstNonlinearFactorGraph(graph: ..., constrainFirst: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedFirstSymbolicFactorGraph(graph: ..., constrainFirst: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedLastDiscreteFactorGraph(graph: ..., constrainLast: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedLastGaussianFactorGraph(graph: ..., constrainLast: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedLastHybridGaussianFactorGraph(graph: ..., constrainLast: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedLastNonlinearFactorGraph(graph: ..., constrainLast: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdConstrainedLastSymbolicFactorGraph(graph: ..., constrainLast: list[int], forceOrder: bool = False) -> Ordering:
        ...
    @staticmethod
    def ColamdDiscreteFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def ColamdGaussianFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def ColamdHybridGaussianFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def ColamdNonlinearFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def ColamdSymbolicFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def CreateDiscreteFactorGraph(orderingType: ..., graph: ...) -> Ordering:
        ...
    @staticmethod
    def CreateGaussianFactorGraph(orderingType: ..., graph: ...) -> Ordering:
        ...
    @staticmethod
    def CreateHybridGaussianFactorGraph(orderingType: ..., graph: ...) -> Ordering:
        ...
    @staticmethod
    def CreateNonlinearFactorGraph(orderingType: ..., graph: ...) -> Ordering:
        ...
    @staticmethod
    def CreateSymbolicFactorGraph(orderingType: ..., graph: ...) -> Ordering:
        ...
    @staticmethod
    def MetisDiscreteFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def MetisGaussianFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def MetisHybridGaussianFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def MetisNonlinearFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def MetisSymbolicFactorGraph(graph: ...) -> Ordering:
        ...
    @staticmethod
    def NaturalDiscreteFactorGraph(fg: ...) -> Ordering:
        ...
    @staticmethod
    def NaturalGaussianFactorGraph(fg: ...) -> Ordering:
        ...
    @staticmethod
    def NaturalHybridGaussianFactorGraph(fg: ...) -> Ordering:
        ...
    @staticmethod
    def NaturalNonlinearFactorGraph(fg: ...) -> Ordering:
        ...
    @staticmethod
    def NaturalSymbolicFactorGraph(fg: ...) -> Ordering:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, keys: list[int]) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def at(self, i: int) -> int:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: Ordering, tol: float) -> bool:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def push_back(self, key: int) -> None:
        ...
    def serialize(self) -> str:
        ...
    def size(self) -> int:
        ...
class OrientedPlane3:
    @staticmethod
    def Dim() -> int:
        """
        Dimensionality of tangent space = 3 DOF.
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, n: Unit3, d: float) -> None:
        ...
    @typing.overload
    def __init__(self, vec: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, a: float, b: float, c: float, d: float) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def dim(self) -> int:
        """
        Dimensionality of tangent space = 3 DOF.
        """
    @typing.overload
    def distance(self) -> float:
        """
        Return the perpendicular distance to the origin.
        """
    @typing.overload
    def distance(self, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        """
        Return the perpendicular distance to the origin.
        """
    def equals(self, s: OrientedPlane3, tol: float = 1e-09) -> bool:
        """
        The equals function with tolerance.
        """
    @typing.overload
    def errorVector(self, other: OrientedPlane3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Computes the error between the two planes, with derivatives. 
        other: the other plane
        """
    @typing.overload
    def errorVector(self, other: OrientedPlane3, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Computes the error between the two planes, with derivatives. 
        other: the other plane
        """
    def localCoordinates(self, s: OrientedPlane3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        The local coordinates function.
        """
    @typing.overload
    def normal(self) -> Unit3:
        """
        Return the normal.
        """
    @typing.overload
    def normal(self, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Unit3:
        """
        Return the normal.
        """
    def planeCoefficients(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Returns the plane coefficients.
        """
    def print(self, s: str = '') -> None:
        """
        The print function.
        """
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> OrientedPlane3:
        """
        The retract function.
        """
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> OrientedPlane3:
        """
        The retract function.
        """
    @typing.overload
    def transform(self, xr: Pose3) -> OrientedPlane3:
        """
        Transforms a plane to the specified pose. 
        xr: a transformation in current coordiante
        Returns: the transformed plane
        """
    @typing.overload
    def transform(self, xr: Pose3, Hp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hr: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> OrientedPlane3:
        """
        Transforms a plane to the specified pose. 
        xr: a transformation in current coordiante
        Hp: optional Jacobian wrpt the destination plane
        Hr: optional jacobian wrpt the pose transformation
        Returns: the transformed plane
        """
class OrientedPlane3DirectionPrior(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Gaussian) -> None:
        ...
    def __repr__(self, s: str = 'OrientedPlane3DirectionPrior', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, expected: NonlinearFactor, tol: float = 1e-09) -> bool:
        """
        equals
        """
    def evaluateError(self, plane: OrientedPlane3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = 'OrientedPlane3DirectionPrior', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class OrientedPlane3Factor(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Gaussian, poseKey: int, landmarkKey: int) -> None:
        ...
    def __repr__(self, s: str = 'OrientedPlane3Factor', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, pose: Pose3, plane: OrientedPlane3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = 'OrientedPlane3Factor', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class PCGSolverParameters(ConjugateGradientParameters):
    preconditioner: PreconditionerParameters
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, preconditioner: PreconditionerParameters) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def print(self, s: str = '') -> None:
        ...
class PinholeCameraCal3Bundler:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def Level(K: Cal3Bundler, pose: Pose2, height: float) -> PinholeCameraCal3Bundler:
        ...
    @staticmethod
    @typing.overload
    def Level(pose: Pose2, height: float) -> PinholeCameraCal3Bundler:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3Bundler) -> PinholeCameraCal3Bundler:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholeCameraCal3Bundler) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3Bundler) -> None:
        ...
    def __repr__(self, s: str = 'PinholeCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3Bundler:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholeCameraCal3Bundler, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: PinholeCameraCal3Bundler) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholeCamera') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def reprojectionError(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholeCameraCal3Bundler:
        ...
    def serialize(self) -> str:
        ...
class PinholeCameraCal3DS2:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def Level(K: Cal3DS2, pose: Pose2, height: float) -> PinholeCameraCal3DS2:
        ...
    @staticmethod
    @typing.overload
    def Level(pose: Pose2, height: float) -> PinholeCameraCal3DS2:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3DS2) -> PinholeCameraCal3DS2:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholeCameraCal3DS2) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3DS2) -> None:
        ...
    def __repr__(self, s: str = 'PinholeCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3DS2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholeCameraCal3DS2, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: PinholeCameraCal3DS2) -> numpy.ndarray[tuple[typing.Literal[15], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholeCamera') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def reprojectionError(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholeCameraCal3DS2:
        ...
    def serialize(self) -> str:
        ...
class PinholeCameraCal3Fisheye:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def Level(K: Cal3Fisheye, pose: Pose2, height: float) -> PinholeCameraCal3Fisheye:
        ...
    @staticmethod
    @typing.overload
    def Level(pose: Pose2, height: float) -> PinholeCameraCal3Fisheye:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3Fisheye) -> PinholeCameraCal3Fisheye:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholeCameraCal3Fisheye) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3Fisheye) -> None:
        ...
    def __repr__(self, s: str = 'PinholeCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3Fisheye:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholeCameraCal3Fisheye, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: PinholeCameraCal3Fisheye) -> numpy.ndarray[tuple[typing.Literal[15], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholeCamera') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def reprojectionError(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholeCameraCal3Fisheye:
        ...
    def serialize(self) -> str:
        ...
class PinholeCameraCal3Unified:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def Level(K: Cal3Unified, pose: Pose2, height: float) -> PinholeCameraCal3Unified:
        ...
    @staticmethod
    @typing.overload
    def Level(pose: Pose2, height: float) -> PinholeCameraCal3Unified:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3Unified) -> PinholeCameraCal3Unified:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholeCameraCal3Unified) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3Unified) -> None:
        ...
    def __repr__(self, s: str = 'PinholeCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3Unified:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholeCameraCal3Unified, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: PinholeCameraCal3Unified) -> numpy.ndarray[tuple[typing.Literal[16], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholeCamera') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def reprojectionError(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholeCameraCal3Unified:
        ...
    def serialize(self) -> str:
        ...
class PinholeCameraCal3_S2:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def Level(K: Cal3_S2, pose: Pose2, height: float) -> PinholeCameraCal3_S2:
        ...
    @staticmethod
    @typing.overload
    def Level(pose: Pose2, height: float) -> PinholeCameraCal3_S2:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3_S2) -> PinholeCameraCal3_S2:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholeCameraCal3_S2) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3_S2) -> None:
        ...
    def __repr__(self, s: str = 'PinholeCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3_S2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholeCameraCal3_S2, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: PinholeCameraCal3_S2) -> numpy.ndarray[tuple[typing.Literal[11], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholeCamera') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def reprojectionError(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholeCameraCal3_S2:
        ...
    def serialize(self) -> str:
        ...
class PinholeCameraCal3f:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    @typing.overload
    def Level(K: Cal3f, pose: Pose2, height: float) -> PinholeCameraCal3f:
        ...
    @staticmethod
    @typing.overload
    def Level(pose: Pose2, height: float) -> PinholeCameraCal3f:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3f) -> PinholeCameraCal3f:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholeCameraCal3f) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3f) -> None:
        ...
    def __repr__(self, s: str = 'PinholeCamera') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3f:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholeCameraCal3f, tol: float) -> bool:
        ...
    def localCoordinates(self, T2: PinholeCameraCal3f) -> numpy.ndarray[tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholeCamera') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def reprojectionError(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholeCameraCal3f:
        ...
    def serialize(self) -> str:
        ...
class PinholePoseCal3Bundler:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    def Level(pose: Pose2, height: float) -> PinholePoseCal3Bundler:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3Bundler) -> PinholePoseCal3Bundler:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholePoseCal3Bundler) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3Bundler) -> None:
        ...
    def __repr__(self, s: str = 'PinholePose') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3Bundler:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholePoseCal3Bundler, tol: float) -> bool:
        ...
    def localCoordinates(self, p: PinholePoseCal3Bundler) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholePose') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholePoseCal3Bundler:
        ...
    def serialize(self) -> str:
        ...
class PinholePoseCal3DS2:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    def Level(pose: Pose2, height: float) -> PinholePoseCal3DS2:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3DS2) -> PinholePoseCal3DS2:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholePoseCal3DS2) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3DS2) -> None:
        ...
    def __repr__(self, s: str = 'PinholePose') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3DS2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholePoseCal3DS2, tol: float) -> bool:
        ...
    def localCoordinates(self, p: PinholePoseCal3DS2) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholePose') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholePoseCal3DS2:
        ...
    def serialize(self) -> str:
        ...
class PinholePoseCal3Fisheye:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    def Level(pose: Pose2, height: float) -> PinholePoseCal3Fisheye:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3Fisheye) -> PinholePoseCal3Fisheye:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholePoseCal3Fisheye) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3Fisheye) -> None:
        ...
    def __repr__(self, s: str = 'PinholePose') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3Fisheye:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholePoseCal3Fisheye, tol: float) -> bool:
        ...
    def localCoordinates(self, p: PinholePoseCal3Fisheye) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholePose') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholePoseCal3Fisheye:
        ...
    def serialize(self) -> str:
        ...
class PinholePoseCal3Unified:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    def Level(pose: Pose2, height: float) -> PinholePoseCal3Unified:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3Unified) -> PinholePoseCal3Unified:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholePoseCal3Unified) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3Unified) -> None:
        ...
    def __repr__(self, s: str = 'PinholePose') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3Unified:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholePoseCal3Unified, tol: float) -> bool:
        ...
    def localCoordinates(self, p: PinholePoseCal3Unified) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholePose') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholePoseCal3Unified:
        ...
    def serialize(self) -> str:
        ...
class PinholePoseCal3_S2:
    @staticmethod
    def Dim() -> int:
        ...
    @staticmethod
    def Level(pose: Pose2, height: float) -> PinholePoseCal3_S2:
        ...
    @staticmethod
    def Lookat(eye: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], target: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], upVector: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], K: Cal3_S2) -> PinholePoseCal3_S2:
        ...
    @staticmethod
    def Project(cameraPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: PinholePoseCal3_S2) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3_S2) -> None:
        ...
    def __repr__(self, s: str = 'PinholePose') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def backproject(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], depth: float, Dresult_dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_ddepth: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dresult_dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def calibration(self) -> Cal3_S2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        ...
    def equals(self, camera: PinholePoseCal3_S2, tol: float) -> bool:
        ...
    def localCoordinates(self, p: PinholePoseCal3_S2) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def pose(self) -> Pose3:
        ...
    def print(self, s: str = 'PinholePose') -> None:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dcal: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def projectSafe(self, pw: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bool]:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3) -> float:
        ...
    @typing.overload
    def range(self, pose: Pose3, Dcamera: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Dpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        ...
    def retract(self, d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> PinholePoseCal3_S2:
        ...
    def serialize(self) -> str:
        ...
class PlanarProjectionFactor1(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, landmark: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bTc: Pose3, calib: Cal3DS2, model: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class PlanarProjectionFactor2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, landmarkKey: int, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], bTc: Pose3, calib: Cal3DS2, model: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class PlanarProjectionFactor3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, poseKey: int, offsetKey: int, calibKey: int, landmark: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def serialize(self) -> str:
        ...
class Pose2:
    @staticmethod
    @typing.overload
    def Align(abPointPairs: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]]) -> Pose2 | None:
        """
        Create Pose2 by aligning two point pairs A pose aTb is estimated between pairs (a_point, b_point) such that a_point = aTb * b_point Note this allows for noise on the points but in that case the mapping will not be exact.
        """
    @staticmethod
    @typing.overload
    def Align(a: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose2 | None:
        ...
    @staticmethod
    @typing.overload
    def Expmap(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Pose2:
        """
        Exponential map at identity - create a rotation from canonical coordinates $ [T_x,T_y,\\theta] $.
        """
    @staticmethod
    @typing.overload
    def Expmap(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose2:
        """
        Exponential map at identity - create a rotation from canonical coordinates $ [T_x,T_y,\\theta] $.
        """
    @staticmethod
    def ExpmapDerivative(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Derivative of Expmap.
        """
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Hat maps from tangent vector to Lie algebra.
        """
    @staticmethod
    def Identity() -> Pose2:
        """
        identity for group operation
        """
    @staticmethod
    @typing.overload
    def Logmap(p: Pose2) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at identity - return the canonical coordinates $ [T_x,T_y,\\theta] $ of this rotation.
        """
    @staticmethod
    @typing.overload
    def Logmap(p: Pose2, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at identity - return the canonical coordinates $ [T_x,T_y,\\theta] $ of this rotation.
        """
    @staticmethod
    def LogmapDerivative(v: Pose2) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Derivative of Logmap.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    @staticmethod
    def adjointMap_(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def adjointTranspose(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
        """
    @staticmethod
    def adjoint_(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def Adjoint(self, xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Apply AdjointMap to twist xi.
        """
    def AdjointMap(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Calculate Adjoint map Ad_pose is 3*3 matrix that when applied to twist xi $ [T_x,T_y,\\theta] $, returns Ad_pose(xi)
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: Pose2) -> None:
        ...
    @typing.overload
    def __init__(self, x: float, y: float, theta: float) -> None:
        ...
    @typing.overload
    def __init__(self, theta: float, t: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, r: Rot2, t: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __mul__(self, arg0: Pose2) -> Pose2:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def bearing(self, point: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot2:
        """
        Calculate bearing to a landmark. 
        point: 2D location of landmark
        Returns: 2D rotation
        """
    @typing.overload
    def between(self, p2: Pose2) -> Pose2:
        ...
    @typing.overload
    def between(self, p2: Pose2, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose2:
        ...
    @typing.overload
    def compose(self, p2: Pose2) -> Pose2:
        ...
    @typing.overload
    def compose(self, p2: Pose2, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, pose: Pose2, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    @typing.overload
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Pose2:
        ...
    @typing.overload
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose2:
        ...
    def inverse(self) -> Pose2:
        """
        inverse
        """
    @typing.overload
    def localCoordinates(self, p: Pose2) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def localCoordinates(self, p: Pose2, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def logmap(self, p: Pose2) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def logmap(self, p: Pose2, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    def range(self, point: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        """
        Calculate range to a landmark. 
        point: 2D location of landmark
        Returns: range (double)
        """
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Pose2:
        ...
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose2:
        ...
    @typing.overload
    def rotation(self) -> Rot2:
        """
        rotation
        """
    @typing.overload
    def rotation(self, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Rot2:
        """
        rotation
        """
    def serialize(self) -> str:
        ...
    def theta(self) -> float:
        """
        get theta
        """
    @typing.overload
    def transformFrom(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def transformFrom(self, points: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        transform many points in Pose coordinates and transform to world. 
        points: 2*N matrix in Pose coordinates
        Returns: points in world coordinates, as 2*N Matrix
        """
    @typing.overload
    def transformTo(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def transformTo(self, points: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        transform many points in world coordinates and transform to Pose. 
        points: 2*N matrix in world coordinates
        Returns: points in Pose coordinates, as 2*N Matrix
        """
    @typing.overload
    def translation(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        translation
        """
    @typing.overload
    def translation(self, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        translation
        """
    def x(self) -> float:
        """
        get x
        """
    def y(self) -> float:
        """
        get y
        """
class Pose3:
    @staticmethod
    @typing.overload
    def Align(abPointPairs: list[tuple[numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]]]) -> Pose3 | None:
        """
        Create Pose3 by aligning two point pairs A pose aTb is estimated between pairs (a_point, b_point) such that a_point = aTb * b_point Note this allows for noise on the points but in that case the mapping will not be exact.
        """
    @staticmethod
    @typing.overload
    def Align(a: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], b: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3 | None:
        ...
    @staticmethod
    @typing.overload
    def Expmap(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Pose3:
        """
        Exponential map at identity - create a rotation from canonical coordinates $ [R_x,R_y,R_z,T_x,T_y,T_z] $.
        """
    @staticmethod
    @typing.overload
    def Expmap(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], Hxi: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        """
        Exponential map at identity - create a rotation from canonical coordinates $ [R_x,R_y,R_z,T_x,T_y,T_z] $.
        """
    @staticmethod
    def ExpmapDerivative(xi: numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        """
        Derivative of Expmap.
        """
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        """
        Hat for Pose3: 
        xi: 6-dim twist (omega,v) where omega = (wx,wy,wz) 3D angular velocity v (vx,vy,vz) = 3D velocity
        Returns: xihat, 4*4 element of Lie algebra that can be exponentiated
        """
    @staticmethod
    def Identity() -> Pose3:
        """
        identity for group operation
        """
    @staticmethod
    @typing.overload
    def Logmap(p: Pose3) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    @typing.overload
    def Logmap(pose: Pose3, Hpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at identity - return the canonical coordinates $ [R_x,R_y,R_z,T_x,T_y,T_z] $ of this rotation.
        """
    @staticmethod
    @typing.overload
    def LogmapDerivative(xi: numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        """
        Derivative of Logmap.
        """
    @staticmethod
    @typing.overload
    def LogmapDerivative(xi: Pose3) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        """
        Derivative of Logmap.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    @staticmethod
    def adjoint(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Action of the adjointMap on a Lie-algebra vector y, with optional derivatives.
        """
    @staticmethod
    def adjointMap(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        """
        Compute the [ad(w,v)] operator as defined in [Kobilarov09siggraph], pg 11 [ad(w,v)] = [w^, zero3; v^, w^] Note that this is the matrix representation of the adjoint operator for se3 Lie algebra, aka the Lie bracket, and also the derivative of Adjoint map for the Lie group SE3. 
        Let $ \\hat{\\xi}_i $ be the se3 Lie algebra, and $ \\hat{\\xi}_i^\\vee = \\xi_i = [\\omega_i,v_i] \\in \\mathbb{R}^6$ be its vector representation. We have the following relationship: $ [\\hat{\\xi}_1,\\hat{\\xi}_2]^\\vee = ad_{\\xi_1}(\\xi_2) = [ad_{(\\omega_1,v_1)}]*\\xi_2 $ We use this to compute the discrete version of the inverse right-trivialized tangent map, and its inverse transpose in the discrete Euler Poincare' (DEP) operator.
        """
    @staticmethod
    def adjointMap_(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def adjointTranspose(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
        """
    @staticmethod
    def adjoint_(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def Adjoint(self, xi_b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Apply this pose's AdjointMap Ad_g to a twist $ \\xi_b $, i.e. 
        a body-fixed velocity, transforming it to the spatial frame $ \\xi^s = g*\\xi^b*g^{-1} = Ad_g * \\xi^b $ Note that H_xib = AdjointMap()
        """
    @typing.overload
    def Adjoint(self, xi_b: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H_this: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H_xib: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Apply this pose's AdjointMap Ad_g to a twist $ \\xi_b $, i.e. 
        a body-fixed velocity, transforming it to the spatial frame $ \\xi^s = g*\\xi^b*g^{-1} = Ad_g * \\xi^b $ Note that H_xib = AdjointMap()
        """
    def AdjointMap(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        """
        Calculate Adjoint map, transforming a twist in this pose's (i.e, body) frame to the world spatial frame Ad_pose is 6*6 matrix that when applied to twist xi $ [R_x,R_y,R_z,T_x,T_y,T_z] $, returns Ad_pose(xi)
        """
    @typing.overload
    def AdjointTranspose(self, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        The dual version of Adjoint.
        """
    @typing.overload
    def AdjointTranspose(self, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H_this: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H_x: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        The dual version of Adjoint.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, r: Rot3, t: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, pose2: Pose2) -> None:
        ...
    @typing.overload
    def __init__(self, mat: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __mul__(self, arg0: Pose3) -> Pose3:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def between(self, pose: Pose3) -> Pose3:
        ...
    @typing.overload
    def between(self, pose: Pose3, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def compose(self, pose: Pose3) -> Pose3:
        ...
    @typing.overload
    def compose(self, pose: Pose3, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, pose: Pose3, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    @typing.overload
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def inverse(self) -> Pose3:
        """
        inverse transformation with derivatives
        """
    @typing.overload
    def inverse(self, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def localCoordinates(self, pose: Pose3) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def localCoordinates(self, pose: Pose3, Hxi: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def logmap(self, p: Pose3) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def logmap(self, p: Pose3, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        """
        convert to 4*4 matrix
        """
    def print(self, s: str = '') -> None:
        """
        print with optional string
        """
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        """
        Calculate range to a landmark. 
        point: 3D location of landmark
        Returns: range (double)
        """
    @typing.overload
    def range(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        """
        Calculate range to a landmark. 
        point: 3D location of landmark
        Returns: range (double)
        """
    @typing.overload
    def range(self, pose: Pose3) -> float:
        """
        Calculate range to another pose. 
        pose: Other SO(3) pose
        Returns: range (double)
        """
    @typing.overload
    def range(self, pose: Pose3, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hpose: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        """
        Calculate range to another pose. 
        pose: Other SO(3) pose
        Returns: range (double)
        """
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], Hxi: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def rotation(self) -> Rot3:
        """
        get rotation
        """
    @typing.overload
    def rotation(self, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Rot3:
        """
        get rotation
        """
    def serialize(self) -> str:
        ...
    @typing.overload
    def slerp(self, t: float, other: Pose3) -> Pose3:
        """
        Spherical Linear interpolation between *this and other. 
        s: a value between 0 and 1.5
        other: final point of interpolation geodesic on manifold
        """
    @typing.overload
    def slerp(self, t: float, other: Pose3, Hx: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hy: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        """
        Spherical Linear interpolation between *this and other. 
        s: a value between 0 and 1.5
        other: final point of interpolation geodesic on manifold
        """
    @typing.overload
    def transformFrom(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        takes point in Pose coordinates and transforms it to world coordinates 
        point: point in Pose coordinates
        Returns: point in world coordinates
        """
    @typing.overload
    def transformFrom(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        takes point in Pose coordinates and transforms it to world coordinates 
        point: point in Pose coordinates
        Hself: optional 3*6 Jacobian wrpt this pose
        Hpoint: optional 3*3 Jacobian wrpt point
        Returns: point in world coordinates
        """
    @typing.overload
    def transformFrom(self, points: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        transform many points in Pose coordinates and transform to world. 
        points: 3*N matrix in Pose coordinates
        Returns: points in world coordinates, as 3*N Matrix
        """
    @typing.overload
    def transformPoseFrom(self, aTb: Pose3) -> Pose3:
        """
        Assuming self == wTa, takes a pose aTb in local coordinates and transforms it to world coordinates wTb = wTa * aTb. 
        This is identical to compose.
        """
    @typing.overload
    def transformPoseFrom(self, pose: Pose3, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], HaTb: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def transformPoseTo(self, wTb: Pose3) -> Pose3:
        """
        Assuming self == wTa, takes a pose wTb in world coordinates and transforms it to local coordinates aTb = inv(wTa) * wTb.
        """
    @typing.overload
    def transformPoseTo(self, pose: Pose3, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], HwTb: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Pose3:
        ...
    @typing.overload
    def transformTo(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        takes point in world coordinates and transforms it to Pose coordinates 
        point: point in world coordinates
        Returns: point in Pose coordinates
        """
    @typing.overload
    def transformTo(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hpoint: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        takes point in world coordinates and transforms it to Pose coordinates 
        point: point in world coordinates
        Hself: optional 3*6 Jacobian wrpt this pose
        Hpoint: optional 3*3 Jacobian wrpt point
        Returns: point in Pose coordinates
        """
    @typing.overload
    def transformTo(self, points: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        transform many points in world coordinates and transform to Pose. 
        points: 3*N matrix in world coordinates
        Returns: points in Pose coordinates, as 3*N Matrix
        """
    @typing.overload
    def translation(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        get translation
        """
    @typing.overload
    def translation(self, Hself: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        get translation
        """
    def x(self) -> float:
        """
        get x
        """
    def y(self) -> float:
        """
        get y
        """
    def z(self) -> float:
        """
        get z
        """
class Pose3AttitudeFactor(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, key: int, nRef: Unit3, model: noiseModel.Diagonal, bMeasured: Unit3) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, nRef: Unit3, model: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def bMeasured(self) -> Unit3:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nTb: Pose3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def nRef(self) -> Unit3:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class PoseRotationPrior2D(NoiseModelFactor):
    @typing.overload
    def __init__(self, key: int, rot_z: Rot2, model: noiseModel.Base) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, pose_z: Pose2, model: noiseModel.Base) -> None:
        ...
    def measured(self) -> Rot2:
        ...
class PoseRotationPrior3D(NoiseModelFactor):
    @typing.overload
    def __init__(self, key: int, rot_z: Rot3, model: noiseModel.Base) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, pose_z: Pose3, model: noiseModel.Base) -> None:
        ...
    def measured(self) -> Rot3:
        ...
class PoseTranslationPrior2D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, key: int, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, pose_z: Pose2, model: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class PoseTranslationPrior3D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, key: int, measured: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, pose_z: Pose3, model: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class PreconditionerParameters:
    def __init__(self) -> None:
        ...
class PreintegratedAhrsMeasurements:
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, params: PreintegrationParams, biasHat: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, p: PreintegrationParams, bias_hat: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], deltaTij: float, deltaRij: Rot3, delRdelBiasOmega: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], preint_meas_cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, rhs: PreintegratedAhrsMeasurements) -> None:
        ...
    def __repr__(self, s: str = 'Preintegrated Measurements: ') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def biasHat(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deltaRij(self) -> Rot3:
        ...
    def deltaTij(self) -> float:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: PreintegratedAhrsMeasurements, tol: float) -> bool:
        """
        equals
        """
    def integrateMeasurement(self, measuredOmega: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], deltaT: float) -> None:
        """
        Add a single gyroscope measurement to the preintegration. 
        Measurements are taken to be in the sensor frame and conversion to the body frame is handled by body_P_sensor in PreintegratedRotationParams (if provided). measuredOmega: Measured angular velocity (as given by the sensor)
        deltaT: Time step
        """
    def print(self, s: str = 'Preintegrated Measurements: ') -> None:
        """
        print
        """
    def resetIntegration(self) -> None:
        """
        Reset integrated quantities to zero.
        """
    def serialize(self) -> str:
        ...
class PreintegratedCombinedMeasurements:
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, params: PreintegrationCombinedParams) -> None:
        ...
    @typing.overload
    def __init__(self, params: PreintegrationCombinedParams, bias: imuBias.ConstantBias) -> None:
        ...
    def __repr__(self, s: str = 'Preintegrated Measurements:') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def biasHat(self) -> imuBias.ConstantBias:
        ...
    def biasHatVector(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deltaPij(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deltaRij(self) -> Rot3:
        ...
    def deltaTij(self) -> float:
        ...
    def deltaVij(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: PreintegratedCombinedMeasurements, tol: float) -> bool:
        """
        equals
        """
    def integrateMeasurement(self, measuredAcc: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], measuredOmega: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], deltaT: float) -> None:
        ...
    def predict(self, state_i: NavState, bias: imuBias.ConstantBias) -> NavState:
        ...
    def preintMeasCov(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = 'Preintegrated Measurements:') -> None:
        ...
    def resetIntegration(self) -> None:
        """
        Re-initialize PreintegratedCombinedMeasurements.
        """
    def resetIntegrationAndSetBias(self, biasHat: imuBias.ConstantBias) -> None:
        ...
    def serialize(self) -> str:
        ...
class PreintegratedImuMeasurements:
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, params: PreintegrationParams) -> None:
        ...
    @typing.overload
    def __init__(self, params: PreintegrationParams, bias: imuBias.ConstantBias) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def biasHat(self) -> imuBias.ConstantBias:
        ...
    def biasHatVector(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deltaPij(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deltaRij(self) -> Rot3:
        ...
    def deltaTij(self) -> float:
        ...
    def deltaVij(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: PreintegratedImuMeasurements, tol: float) -> bool:
        """
        equals
        """
    def integrateMeasurement(self, measuredAcc: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], measuredOmega: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], deltaT: float) -> None:
        ...
    def predict(self, state_i: NavState, bias: imuBias.ConstantBias) -> NavState:
        ...
    def preintMeasCov(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        """
        Return pre-integrated measurement covariance.
        """
    def preintegrated(self) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        """
        print
        """
    def resetIntegration(self) -> None:
        """
        Re-initialize PreintegratedImuMeasurements.
        """
    def resetIntegrationAndSetBias(self, biasHat: imuBias.ConstantBias) -> None:
        ...
    def serialize(self) -> str:
        ...
class PreintegratedRotation:
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, params: PreintegratedRotationParams) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def biascorrectedDeltaRij(self, biasOmegaIncr: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        """
        Return a bias corrected version of the integrated rotation. 
        biasOmegaIncr: An increment with respect to biasHat used above.
        """
    def delRdelBiasOmega(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def deltaRij(self) -> Rot3:
        ...
    def deltaTij(self) -> float:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: PreintegratedRotation, tol: float) -> bool:
        ...
    def integrateCoriolis(self, rot_i: Rot3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Integrate coriolis correction in body frame rot_i.
        """
    def integrateGyroMeasurement(self, measuredOmega: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], biasHat: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], deltaT: float) -> None:
        """
        Calculate an incremental rotation given the gyro measurement and a time interval, and update both deltaTij_ and deltaRij_. 
        measuredOmega: The measured angular velocity (as given by the sensor)
        bias: The biasHat estimate
        deltaT: The time interval
        """
    def print(self, s: str = '') -> None:
        ...
    def resetIntegration(self) -> None:
        """
        Re-initialize PreintegratedMeasurements.
        """
    def serialize(self) -> str:
        ...
class PreintegratedRotationParams:
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: PreintegratedRotationParams, tol: float) -> bool:
        ...
    def getBodyPSensor(self) -> Pose3 | None:
        ...
    def getGyroscopeCovariance(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def getOmegaCoriolis(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]] | None:
        ...
    def print(self, s: str = '') -> None:
        ...
    def serialize(self) -> str:
        ...
    def setBodyPSensor(self, pose: Pose3) -> None:
        ...
    def setGyroscopeCovariance(self, cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setOmegaCoriolis(self, omega: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
class PreintegrationCombinedParams(PreintegrationParams):
    @staticmethod
    @typing.overload
    def MakeSharedD(g: float) -> PreintegrationCombinedParams:
        ...
    @staticmethod
    @typing.overload
    def MakeSharedD() -> PreintegrationCombinedParams:
        ...
    @staticmethod
    @typing.overload
    def MakeSharedU(g: float) -> PreintegrationCombinedParams:
        ...
    @staticmethod
    @typing.overload
    def MakeSharedU() -> PreintegrationCombinedParams:
        ...
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, n_gravity: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: PreintegrationCombinedParams, tol: float) -> bool:
        ...
    def getBiasAccCovariance(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def getBiasAccOmegaInit(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        ...
    def getBiasOmegaCovariance(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        ...
    def serialize(self) -> str:
        ...
    def setBiasAccCovariance(self, cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setBiasAccOmegaInit(self, cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setBiasOmegaCovariance(self, cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
class PreintegrationParams(PreintegratedRotationParams):
    n_gravity: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    @staticmethod
    @typing.overload
    def MakeSharedD(g: float) -> PreintegrationParams:
        ...
    @staticmethod
    @typing.overload
    def MakeSharedD() -> PreintegrationParams:
        ...
    @staticmethod
    @typing.overload
    def MakeSharedU(g: float) -> PreintegrationParams:
        ...
    @staticmethod
    @typing.overload
    def MakeSharedU() -> PreintegrationParams:
        ...
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, n_gravity: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: PreintegrationParams, tol: float) -> bool:
        ...
    def getAccelerometerCovariance(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def getIntegrationCovariance(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def getUse2ndOrderCoriolis(self) -> bool:
        ...
    def print(self, s: str = '') -> None:
        ...
    def serialize(self) -> str:
        ...
    def setAccelerometerCovariance(self, cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setIntegrationCovariance(self, cov: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setUse2ndOrderCoriolis(self, flag: bool) -> None:
        ...
class PriorFactorCal3Bundler(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Cal3Bundler, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Cal3Bundler:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorCal3DS2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Cal3DS2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Cal3DS2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorCal3Fisheye(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Cal3Fisheye, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Cal3Fisheye:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorCal3Unified(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Cal3Unified, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Cal3Unified:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorCal3_S2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Cal3_S2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Cal3_S2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorCalibratedCamera(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: CalibratedCamera, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> CalibratedCamera:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorConstantBias(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: ..., noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> ...:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorDouble(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorNavState(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: ..., noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> ...:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPinholeCameraCal3Bundler(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: PinholeCameraCal3Bundler, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> PinholeCameraCal3Bundler:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPinholeCameraCal3Fisheye(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: PinholeCameraCal3Fisheye, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> PinholeCameraCal3Fisheye:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPinholeCameraCal3Unified(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: PinholeCameraCal3Unified, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> PinholeCameraCal3Unified:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPinholeCameraCal3_S2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: PinholeCameraCal3_S2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> PinholeCameraCal3_S2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPoint2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPoint3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Pose2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Pose2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorPose3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Pose3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Pose3:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorRot2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Rot2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Rot2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorRot3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Rot3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Rot3:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorSO3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: SO3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> SO3:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorSO4(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: SO4, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> SO4:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorSOn(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: SOn, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> SOn:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorSimilarity2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Similarity2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Similarity2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorSimilarity3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Similarity3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Similarity3:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorStereoPoint2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: StereoPoint2, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> StereoPoint2:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorUnit3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: Unit3, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> Unit3:
        ...
    def serialize(self) -> str:
        ...
class PriorFactorVector(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key: int, prior: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def prior(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def serialize(self) -> str:
        ...
class Quaternion:
    def coeffs(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def w(self) -> float:
        ...
    def x(self) -> float:
        ...
    def y(self) -> float:
        ...
    def z(self) -> float:
        ...
class RangeFactor2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactor2D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactor3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactor3D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorCalibratedCamera(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorCalibratedCameraPoint(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorPose3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorSimpleCamera(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorSimpleCameraPoint(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorWithTransform2D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base, body_T_sensor: Pose2) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorWithTransform3D(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base, body_T_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorWithTransformPose2(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base, body_T_sensor: Pose2) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RangeFactorWithTransformPose3(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, key1: int, key2: int, measured: float, noiseModel: noiseModel.Base, body_T_sensor: Pose3) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def measured(self) -> float:
        ...
    def serialize(self) -> str:
        ...
class RedirectCout:
    def __init__(self) -> None:
        ...
    def str(self) -> str:
        """
        return the string
        """
class ReferenceFrameFactorPoint3Pose3(NoiseModelFactor):
    def __init__(self, globalKey: int, transKey: int, localKey: int, model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, global: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], trans: Pose3, local: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class Rot2:
    @staticmethod
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot2:
        """
        Exponential map at identity - create a rotation from canonical coordinates.
        """
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[2]], numpy.dtype[numpy.float64]]:
        """
        Hat maps from tangent vector to Lie algebra.
        """
    @staticmethod
    def Identity() -> Rot2:
        """
        Identity.
        """
    @staticmethod
    def Logmap(r: Rot2) -> numpy.ndarray[tuple[typing.Literal[1], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at identity - return the canonical coordinates of this rotation.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[1], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    @staticmethod
    def atan2(y: float, x: float) -> Rot2:
        """
        Named constructor that behaves as atan2, i.e., y,x order (!) and normalizes.
        """
    @staticmethod
    def fromAngle(theta: float) -> Rot2:
        """
        Named constructor from angle in radians.
        """
    @staticmethod
    def fromCosSin(c: float, s: float) -> Rot2:
        """
        Named constructor from cos(theta),sin(theta) pair.
        """
    @staticmethod
    def fromDegrees(theta: float) -> Rot2:
        """
        Named constructor from angle in degrees.
        """
    @staticmethod
    def relativeBearing(d: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot2:
        """
        Named constructor with derivative Calculate relative bearing to a landmark in local coordinate frame. 
        d: 2D location of landmark
        Returns: 2D rotation
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, theta: float) -> None:
        ...
    def __mul__(self, arg0: Rot2) -> Rot2:
        ...
    def __repr__(self, s: str = 'theta') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def between(self, p2: Rot2) -> Rot2:
        ...
    def c(self) -> float:
        """
        return cos
        """
    def compose(self, p2: Rot2) -> Rot2:
        ...
    def degrees(self) -> float:
        """
        return angle (DEGREES)
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, R: Rot2, tol: float) -> bool:
        """
        equals with an tolerance
        """
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot2:
        ...
    def inverse(self) -> Rot2:
        """
        The inverse rotation - negative angle.
        """
    @typing.overload
    def localCoordinates(self, p: Rot2) -> numpy.ndarray[tuple[typing.Literal[1], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def localCoordinates(self, p: Rot2, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[1], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def logmap(self, p: Rot2) -> numpy.ndarray[tuple[typing.Literal[1], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[2]], numpy.dtype[numpy.float64]]:
        """
        return 2*2 rotation matrix
        """
    def print(self, s: str = 'theta') -> None:
        """
        print
        """
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot2:
        ...
    @typing.overload
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Rot2:
        ...
    def rotate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        rotate point from rotated coordinate frame to world $ p^w = R_c^w p^c $
        """
    def s(self) -> float:
        """
        return sin
        """
    def serialize(self) -> str:
        ...
    def theta(self) -> float:
        """
        return angle (RADIANS)
        """
    def unrotate(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        rotate point from world to rotated frame $ p^c = (R_c^w)^T p^w $
        """
class Rot3:
    @staticmethod
    def AxisAngle(axis: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], angle: float) -> Rot3:
        """
        Convert from axis/angle representation. 
        axis: is the rotation axis, unit length
        angle: rotation angle
        Returns: incremental rotation
        """
    @staticmethod
    def ClosestTo(M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Rot3:
        """
        Static, named constructor that finds Rot3 element closest to M in Frobenius norm. 
        Uses Full SVD to compute the orthogonal matrix, thus is highly accurate and robust. N. J. Higham. Matrix nearness problems and applications. In M. J. C. Gover and S. Barnett, editors, Applications of Matrix Theory, pages 1–27. Oxford University Press, 1989.
        """
    @staticmethod
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        """
        Exponential map - create a rotation from canonical coordinates $ [R_x,R_y,R_z] $ using Rodrigues' formula.
        """
    @staticmethod
    def ExpmapDerivative(omega: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Hat maps from tangent vector to Lie algebra.
        """
    @staticmethod
    def Identity() -> Rot3:
        """
        identity rotation for group operation
        """
    @staticmethod
    def Logmap(R: Rot3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map - returns the canonical coordinates $ [R_x,R_y,R_z] $ of this rotation.
        """
    @staticmethod
    def LogmapDerivative(omega: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Pitch(t: float) -> Rot3:
        """
        Positive pitch is up (increasing aircraft altitude).See ypr.
        """
    @staticmethod
    def Quaternion(w: float, x: float, y: float, z: float) -> Rot3:
        """
        Create from Quaternion coefficients.
        """
    @staticmethod
    @typing.overload
    def Rodrigues(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        ...
    @staticmethod
    @typing.overload
    def Rodrigues(wx: float, wy: float, wz: float) -> Rot3:
        """
        Rodrigues' formula to compute an incremental rotation. 
        wx: Incremental roll (about X)
        wy: Incremental pitch (about Y)
        wz: Incremental yaw (about Z)
        Returns: incremental rotation
        """
    @staticmethod
    def Roll(t: float) -> Rot3:
        ...
    @staticmethod
    def Rx(t: float) -> Rot3:
        """
        Rotation around X axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
        """
    @staticmethod
    def Ry(t: float) -> Rot3:
        """
        Rotation around Y axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
        """
    @staticmethod
    def Rz(t: float) -> Rot3:
        """
        Rotation around Z axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
        """
    @staticmethod
    @typing.overload
    def RzRyRx(x: float, y: float, z: float) -> Rot3:
        """
        Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
        """
    @staticmethod
    @typing.overload
    def RzRyRx(xyz: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        """
        Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    @staticmethod
    def Yaw(t: float) -> Rot3:
        """
        Positive yaw is to right (as in aircraft heading). See ypr.
        """
    @staticmethod
    def Ypr(y: float, p: float, r: float) -> Rot3:
        """
        Returns rotation nRb from body to nav frame. 
        For vehicle coordinate frame X forward, Y right, Z down: Positive yaw is to right (as in aircraft heading). Positive pitch is up (increasing aircraft altitude). Positive roll is to right (increasing yaw in aircraft). Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw) as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf. For vehicle coordinate frame X forward, Y left, Z up: Positive yaw is to left (as in aircraft heading). Positive pitch is down (decreasing aircraft altitude). Positive roll is to right (decreasing yaw in aircraft).
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, col1: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], col2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], col3: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, R11: float, R12: float, R13: float, R21: float, R22: float, R23: float, R31: float, R32: float, R33: float) -> None:
        ...
    @typing.overload
    def __init__(self, w: float, x: float, y: float, z: float) -> None:
        ...
    def __mul__(self, arg0: Rot3) -> Rot3:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def axisAngle(self) -> tuple[..., float]:
        """
        Compute the Euler axis and angle (in radians) representation of this rotation. 
        The angle is in the range [0, π]. If the angle is not in the range, the axis is flipped around accordingly so that the returned angle is within the specified range. pair consisting of Unit3 axis and angle in radians  Returns: pair consisting of
        """
    def between(self, p2: Rot3) -> Rot3:
        ...
    def compose(self, p2: Rot3) -> Rot3:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, p: Rot3, tol: float) -> bool:
        """
        equals with an tolerance
        """
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        ...
    def inverse(self) -> Rot3:
        """
        inverse of a rotation
        """
    def localCoordinates(self, p: Rot3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def logmap(self, p: Rot3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        return 3*3 rotation matrix
        """
    def pitch(self) -> float:
        """
        Accessor to get to component of angle representations NOTE: these are not efficient to get to multiple separate parts, you should instead use xyz() or ypr() TODO: make this more efficient.
        """
    def print(self, s: str = '') -> None:
        """
        print
        """
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Rot3:
        ...
    def roll(self) -> float:
        """
        Accessor to get to component of angle representations NOTE: these are not efficient to get to multiple separate parts, you should instead use xyz() or ypr() TODO: make this more efficient.
        """
    @typing.overload
    def rotate(self, p: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        rotate point from rotated coordinate frame to world $ p^w = R_c^w p^c $
        """
    @typing.overload
    def rotate(self, p: ...) -> ...:
        """
        rotate 3D direction from rotated coordinate frame to world frame
        """
    @typing.overload
    def rotate(self, p: ..., HR: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], Hp: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> ...:
        """
        rotate 3D direction from rotated coordinate frame to world frame
        """
    def rpy(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Use RQ to calculate roll-pitch-yaw angle representation. 
        a vector containing rpy s.t. R = Rot3::Ypr(y,p,r)  Returns: a vector containing rpy s.t. R = Rot3::Ypr(y,p,r)
        """
    def serialize(self) -> str:
        ...
    def slerp(self, t: float, other: Rot3) -> Rot3:
        """
        Spherical Linear intERPolation between *this and other. 
        t: a value between 0 and 1
        other: final point of interpolation geodesic on manifold
        """
    def toQuaternion(self) -> Quaternion:
        """
        Compute the quaternion representation of this rotation. 
        The quaternion  Returns: The quaternion
        """
    def transpose(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Return 3*3 transpose (inverse) rotation matrix.
        """
    @typing.overload
    def unrotate(self, p: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        rotate point from world to rotated frame $ p^c = (R_c^w)^T p^w $
        """
    @typing.overload
    def unrotate(self, p: ...) -> ...:
        """
        unrotate 3D direction from world frame to rotated coordinate frame
        """
    def xyz(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Use RQ to calculate xyz angle representation. 
        a vector containing x,y,z s.t. R = Rot3::RzRyRx(x,y,z)  Returns: a vector containing x,y,z s.t. R = Rot3::RzRyRx(x,y,z)
        """
    def yaw(self) -> float:
        """
        Accessor to get to component of angle representations NOTE: these are not efficient to get to multiple separate parts, you should instead use xyz() or ypr() TODO: make this more efficient.
        """
    def ypr(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Use RQ to calculate yaw-pitch-roll angle representation. 
        a vector containing ypr s.t. R = Rot3::Ypr(y,p,r)  Returns: a vector containing ypr s.t. R = Rot3::Ypr(y,p,r)
        """
class Rot3AttitudeFactor(NoiseModelFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, key: int, nRef: Unit3, model: noiseModel.Diagonal, bMeasured: Unit3) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, nRef: Unit3, model: noiseModel.Diagonal) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def bMeasured(self) -> Unit3:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: NonlinearFactor, tol: float) -> bool:
        """
        equals
        """
    def evaluateError(self, nRb: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def nRef(self) -> Unit3:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
    def serialize(self) -> str:
        ...
class RotateDirectionsFactor(NoiseModelFactor):
    @staticmethod
    def Initialize(i_p: Unit3, c_z: Unit3) -> Rot3:
        """
        Initialize rotation iRc such that i_p = iRc * c_z.
        """
    def __init__(self, key: int, i_p: Unit3, c_z: Unit3, model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, iRc: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class RotateFactor(NoiseModelFactor):
    def __init__(self, key: int, P: Rot3, Z: Rot3, model: noiseModel.Base) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluateError(self, R: Rot3) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        """
        print
        """
class SO3:
    @staticmethod
    def AxisAngle(axis: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], theta: float) -> SO3:
        ...
    @staticmethod
    def ClosestTo(M: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> SO3:
        ...
    @staticmethod
    def Expmap(v: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO3:
        ...
    @staticmethod
    def ExpmapDerivative(omega: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def FromMatrix(R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> SO3:
        ...
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Identity() -> SO3:
        ...
    @staticmethod
    def Logmap(p: SO3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def LogmapDerivative(omega: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Vee(xi: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __mul__(self, arg0: SO3) -> SO3:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def between(self, R: SO3) -> SO3:
        ...
    def compose(self, R: SO3) -> SO3:
        ...
    def equals(self, other: SO3, tol: float) -> bool:
        ...
    def expmap(self, v: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO3:
        ...
    def inverse(self) -> SO3:
        ...
    def localCoordinates(self, R: SO3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def logmap(self, p: SO3) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        ...
    def retract(self, v: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO3:
        ...
    def vec(self) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class SO4:
    @staticmethod
    @typing.overload
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO4:
        ...
    @staticmethod
    @typing.overload
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO4:
        ...
    @staticmethod
    def FromMatrix(R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> SO4:
        ...
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Identity() -> SO4:
        ...
    @staticmethod
    def Logmap(p: SO4) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Vee(xi: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __mul__(self, arg0: SO4) -> SO4:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def between(self, Q: SO4) -> SO4:
        ...
    def compose(self, Q: SO4) -> SO4:
        ...
    def equals(self, other: SO4, tol: float) -> bool:
        ...
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO4:
        ...
    def inverse(self) -> SO4:
        ...
    def localCoordinates(self, Q: SO4) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def logmap(self, p: SO4) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        ...
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SO4:
        ...
    def vec(self) -> numpy.ndarray[tuple[typing.Literal[16], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class SOn:
    @staticmethod
    @typing.overload
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SOn:
        ...
    @staticmethod
    @typing.overload
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SOn:
        ...
    @staticmethod
    def FromMatrix(R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> SOn:
        ...
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Identity() -> SOn:
        ...
    @staticmethod
    def Lift(n: int, R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> SOn:
        ...
    @staticmethod
    def Logmap(p: SOn) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @staticmethod
    def Vee(xi: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __getstate__(self) -> tuple:
        ...
    def __init__(self, n: int) -> None:
        ...
    def __mul__(self, arg0: SOn) -> SOn:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def between(self, Q: SOn) -> SOn:
        ...
    def compose(self, Q: SOn) -> SOn:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, other: SOn, tol: float) -> bool:
        ...
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SOn:
        ...
    def inverse(self) -> SOn:
        ...
    def localCoordinates(self, Q: SOn) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def logmap(self, p: SOn) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        ...
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SOn:
        ...
    def serialize(self) -> str:
        ...
    def vec(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class Sampler:
    @typing.overload
    def __init__(self, model: noiseModel.Diagonal, seed: int) -> None:
        ...
    @typing.overload
    def __init__(self, sigmas: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], seed: int) -> None:
        ...
    def dim(self) -> int:
        ...
    def model(self) -> noiseModel.Diagonal:
        ...
    def sample(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        sample from distribution
        """
    def sigmas(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class Scenario:
    def acceleration_b(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def acceleration_n(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        acceleration in nav frame
        """
    def navState(self, t: float) -> NavState:
        ...
    def omega_b(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        angular velocity in body frame
        """
    def pose(self, t: float) -> Pose3:
        """
        pose at time t
        """
    def rotation(self, t: float) -> Rot3:
        ...
    def velocity_b(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def velocity_n(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        velocity at time t, in nav frame
        """
class ScenarioRunner:
    def __init__(self, scenario: Scenario, p: PreintegrationParams, imuSampleTime: float, bias: imuBias.ConstantBias) -> None:
        ...
    def actualAngularVelocity(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def actualSpecificForce(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def estimateCovariance(self, T: float, N: int, estimatedBias: imuBias.ConstantBias) -> numpy.ndarray[tuple[typing.Literal[9], typing.Literal[9]], numpy.dtype[numpy.float64]]:
        """
        Compute a Monte Carlo estimate of the predict covariance using N samples.
        """
    def estimateNoiseCovariance(self, N: int) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        """
        Estimate covariance of sampled noise for sanity-check.
        """
    def gravity_n(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def imuSampleTime(self) -> float:
        ...
    def integrate(self, T: float, estimatedBias: imuBias.ConstantBias, corrupted: bool) -> PreintegratedImuMeasurements:
        """
        Integrate measurements for T seconds into a PIM.
        """
    def measuredAngularVelocity(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measuredSpecificForce(self, t: float) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def predict(self, pim: PreintegratedImuMeasurements, estimatedBias: imuBias.ConstantBias) -> NavState:
        """
        Predict predict given a PIM.
        """
class SfmData:
    @staticmethod
    def FromBalFile(filename: str) -> SfmData:
        """
        Parse a "Bundle Adjustment in the Large" (BAL) file and return result as SfmData instance. 
        filename: The name of the BAL file.
        Returns: SfM structure where the data is stored.
        """
    @staticmethod
    def FromBundlerFile(filename: str) -> SfmData:
        """
        Parses a bundler output file and return result as SfmData instance. 
        filename: The name of the bundler file
        data: SfM structure where the data is stored
        Returns: true if the parsing was successful, false otherwise
        """
    def __getstate__(self) -> tuple:
        ...
    def __init__(self) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def addCamera(self, cam: PinholeCameraCal3Bundler) -> None:
        """
        Add a camera to SfmData.
        """
    def addTrack(self, t: SfmTrack) -> None:
        """
        Add a track to SfmData.
        """
    def camera(self, idx: int) -> PinholeCameraCal3Bundler:
        """
        The camera pose at frame index idx
        """
    def cameraList(self) -> list[PinholeCameraCal3Bundler]:
        """
        Getters.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, sfmData: SfmData, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def generalSfmFactors(self, model: noiseModel.Base = ...) -> NonlinearFactorGraph:
        """
        Create projection factors using keys i and P(j) 
        model: a noise model for projection errors
        """
    def numberCameras(self) -> int:
        """
        The number of cameras.
        """
    def numberTracks(self) -> int:
        """
        The number of reconstructed 3D points.
        """
    def serialize(self) -> str:
        ...
    def sfmFactorGraph(self, model: noiseModel.Base = ..., fixedCamera: int = 0, fixedPoint: int = 0) -> NonlinearFactorGraph:
        """
        Create factor graph with projection factors and gauge fix. 
        Note: pose keys are simply integer indices, points use Symbol('p', j). model: a noise model for projection errors
        fixedCamera: which camera to fix, if any (use std::nullopt if none)
        fixedPoint: which point to fix, if any (use std::nullopt if none)
        """
    def track(self, idx: int) -> SfmTrack:
        """
        The track formed by series of landmark measurements.
        """
    def trackList(self) -> list[SfmTrack]:
        ...
class SfmTrack(SfmTrack2d):
    b: float
    g: float
    p: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    r: float
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, pt: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, sfmTrack: SfmTrack, tol: float) -> bool:
        """
        assert equality up to a tolerance
        """
    def point3(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get 3D point.
        """
    def serialize(self) -> str:
        ...
class SfmTrack2d:
    measurements: list[tuple[int, numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]]
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, measurements: list[tuple[int, numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]]) -> None:
        ...
    def addMeasurement(self, idx: int, m: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        """
        Add measurement (camera_idx, Point2) to track.
        """
    def hasUniqueCameras(self) -> bool:
        """
        Check that no two measurements are from the same camera. 
        boolean result of the validation.  Returns: boolean result of the validation.
        """
    def indexVector(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.int32]]:
        """
        Return the camera indices of the measurements.
        """
    def measurement(self, idx: int) -> tuple[int, numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        """
        Get the measurement (camera index, Point2) at pose index idx
        """
    def measurementMatrix(self) -> numpy.ndarray[tuple[M, typing.Literal[2]], numpy.dtype[numpy.float64]]:
        """
        Return the measurements as a 2D matrix.
        """
    def numberMeasurements(self) -> int:
        """
        Total number of measurements in this track.
        """
    def siftIndex(self, idx: int) -> tuple[int, int]:
        """
        Get the SIFT feature index corresponding to the measurement at idx
        """
class ShonanAveraging2:
    @typing.overload
    def __init__(self, g2oFile: str) -> None:
        ...
    @typing.overload
    def __init__(self, g2oFile: str, parameters: ShonanAveragingParameters2) -> None:
        ...
    @typing.overload
    def __init__(self, factors: list[BetweenFactorPose2], parameters: ShonanAveragingParameters2) -> None:
        ...
    def buildGraphAt(self, p: int) -> NonlinearFactorGraph:
        ...
    def checkOptimality(self, values: Values) -> bool:
        ...
    def computeA_(self, values: Values) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def computeLambda_(self, values: Values) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def computeMinEigenValue(self, values: Values) -> float:
        ...
    def computeMinEigenVector(self, values: Values) -> tuple[float, numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def cost(self, values: Values) -> float:
        ...
    def costAt(self, p: int, values: Values) -> float:
        ...
    def createOptimizerAt(self, p: int, initial: Values) -> LevenbergMarquardtOptimizer:
        ...
    def denseD(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def denseL(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def denseQ(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def initializeRandomly(self) -> Values:
        ...
    def initializeRandomlyAt(self, p: int) -> Values:
        ...
    def initializeWithDescent(self, p: int, values: Values, minEigenVector: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], minEigenValue: float) -> Values:
        ...
    def keys(self, i: int) -> list[int]:
        ...
    def measured(self, i: int) -> Rot2:
        ...
    def nrUnknowns(self) -> int:
        ...
    def numberMeasurements(self) -> int:
        ...
    def projectFrom(self, p: int, values: Values) -> Values:
        ...
    def roundSolution(self, values: Values) -> Values:
        ...
    def run(self, initial: Values, min_p: int, max_p: int) -> tuple[Values, float]:
        ...
    def tryOptimizingAt(self, p: int, initial: Values) -> Values:
        ...
class ShonanAveraging3:
    @typing.overload
    def __init__(self, measurements: list[...], parameters: ShonanAveragingParameters3 = ...) -> None:
        ...
    @typing.overload
    def __init__(self, g2oFile: str) -> None:
        ...
    @typing.overload
    def __init__(self, g2oFile: str, parameters: ShonanAveragingParameters3) -> None:
        ...
    @typing.overload
    def __init__(self, factors: list[BetweenFactorPose3], parameters: ShonanAveragingParameters3 = ...) -> None:
        ...
    def buildGraphAt(self, p: int) -> NonlinearFactorGraph:
        ...
    def checkOptimality(self, values: Values) -> bool:
        ...
    def computeA_(self, values: Values) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def computeLambda_(self, values: Values) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def computeMinEigenValue(self, values: Values) -> float:
        ...
    def computeMinEigenVector(self, values: Values) -> tuple[float, numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def cost(self, values: Values) -> float:
        ...
    def costAt(self, p: int, values: Values) -> float:
        ...
    def createOptimizerAt(self, p: int, initial: Values) -> LevenbergMarquardtOptimizer:
        ...
    def denseD(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def denseL(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def denseQ(self) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def initializeRandomly(self) -> Values:
        ...
    def initializeRandomlyAt(self, p: int) -> Values:
        ...
    def initializeWithDescent(self, p: int, values: Values, minEigenVector: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], minEigenValue: float) -> Values:
        ...
    def keys(self, i: int) -> list[int]:
        ...
    def measured(self, i: int) -> Rot3:
        ...
    def nrUnknowns(self) -> int:
        ...
    def numberMeasurements(self) -> int:
        ...
    def projectFrom(self, p: int, values: Values) -> Values:
        ...
    def roundSolution(self, values: Values) -> Values:
        ...
    def run(self, initial: Values, min_p: int, max_p: int) -> tuple[Values, float]:
        ...
    def tryOptimizingAt(self, p: int, initial: Values) -> Values:
        ...
class ShonanAveragingParameters2:
    @typing.overload
    def __init__(self, lm: LevenbergMarquardtParams) -> None:
        ...
    @typing.overload
    def __init__(self, lm: LevenbergMarquardtParams, method: str) -> None:
        ...
    def getAnchor(self) -> tuple[int, Rot2]:
        ...
    def getAnchorWeight(self) -> float:
        ...
    def getCertifyOptimality(self) -> bool:
        ...
    def getGaugesWeight(self) -> float:
        ...
    def getKarcherWeight(self) -> float:
        ...
    def getLMParams(self) -> LevenbergMarquardtParams:
        ...
    def getOptimalityThreshold(self) -> float:
        ...
    def getUseHuber(self) -> bool:
        ...
    def setAnchor(self, index: int, value: Rot2) -> None:
        ...
    def setAnchorWeight(self, value: float) -> None:
        ...
    def setCertifyOptimality(self, value: bool) -> None:
        ...
    def setGaugesWeight(self, value: float) -> None:
        ...
    def setKarcherWeight(self, value: float) -> None:
        ...
    def setOptimalityThreshold(self, value: float) -> None:
        ...
    def setUseHuber(self, value: bool) -> None:
        ...
class ShonanAveragingParameters3:
    @typing.overload
    def __init__(self, lm: LevenbergMarquardtParams) -> None:
        ...
    @typing.overload
    def __init__(self, lm: LevenbergMarquardtParams, method: str) -> None:
        ...
    def getAnchor(self) -> tuple[int, Rot3]:
        ...
    def getAnchorWeight(self) -> float:
        ...
    def getCertifyOptimality(self) -> bool:
        ...
    def getGaugesWeight(self) -> float:
        ...
    def getKarcherWeight(self) -> float:
        ...
    def getLMParams(self) -> LevenbergMarquardtParams:
        ...
    def getOptimalityThreshold(self) -> float:
        ...
    def getUseHuber(self) -> bool:
        ...
    def setAnchor(self, index: int, value: Rot3) -> None:
        ...
    def setAnchorWeight(self, value: float) -> None:
        ...
    def setCertifyOptimality(self, value: bool) -> None:
        ...
    def setGaugesWeight(self, value: float) -> None:
        ...
    def setKarcherWeight(self, value: float) -> None:
        ...
    def setOptimalityThreshold(self, value: float) -> None:
        ...
    def setUseHuber(self, value: bool) -> None:
        ...
class ShonanFactor3(NoiseModelFactor):
    @typing.overload
    def __init__(self, key1: int, key2: int, R12: Rot3, p: int) -> None:
        ...
    @typing.overload
    def __init__(self, key1: int, key2: int, R12: Rot3, p: int, model: noiseModel.Base) -> None:
        ...
    def evaluateError(self, Q1: SOn, Q2: SOn) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class Similarity2:
    @staticmethod
    @typing.overload
    def Align(abPointPairs: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]]) -> Similarity2:
        """
        Create Similarity2 by aligning at least two point pairs.
        """
    @staticmethod
    @typing.overload
    def Align(abPosePairs: list[tuple[Pose2, Pose2]]) -> Similarity2:
        """
        Create the Similarity2 object that aligns at least two pose pairs. 
        Each pair is of the form (aTi, bTi). Given a list of pairs in frame a, and a list of pairs in frame b, Align() will compute the best-fit Similarity2 aSb transformation to align them. First, the rotation aRb will be computed as the average (Karcher mean) of many estimates aRb (from each pair). Afterwards, the scale factor will be computed using the algorithm described here: http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
        """
    @staticmethod
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Similarity2:
        """
        Exponential map at the identity.
        """
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Hat maps from tangent vector to Lie algebra.
        """
    @staticmethod
    def Logmap(S: Similarity2) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at the identity $ [t_x, t_y, \\delta, \\lambda] $.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, s: float) -> None:
        ...
    @typing.overload
    def __init__(self, R: Rot2, t: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], s: float) -> None:
        ...
    @typing.overload
    def __init__(self, R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], t: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], s: float) -> None:
        ...
    @typing.overload
    def __init__(self, T: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def equals(self, sim: Similarity2, tol: float) -> bool:
        """
        Compare with tolerance.
        """
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Similarity2:
        ...
    def logmap(self, p: Similarity2) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Calculate 4*4 matrix group equivalent.
        """
    def print(self, s: str = '') -> None:
        """
        Print with optional string.
        """
    def rotation(self) -> Rot2:
        """
        Return a GTSAM rotation.
        """
    def scale(self) -> float:
        """
        Return the scale.
        """
    @typing.overload
    def transformFrom(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Action on a point p is s*(R*p+t)
        """
    @typing.overload
    def transformFrom(self, T: Pose2) -> Pose2:
        """
        Action on a pose T. 
        |Rs ts| |R t| |Rs*R Rs*t+ts| |0 1/s| * |0 1| = | 0 1/s |, the result is still a Sim2 object. To retrieve a Pose2, we normalized the scale value into 1. |Rs*R Rs*t+ts| |Rs*R s(Rs*t+ts)| | 0 1/s | = | 0 1 | This group action satisfies the compatibility condition. For more details, refer to: https://en.wikipedia.org/wiki/Group_action
        """
    def translation(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return a GTSAM translation.
        """
class Similarity3:
    @staticmethod
    @typing.overload
    def Align(abPointPairs: list[tuple[numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]]]) -> Similarity3:
        """
        Create Similarity3 by aligning at least three point pairs.
        """
    @staticmethod
    @typing.overload
    def Align(abPosePairs: list[tuple[Pose3, Pose3]]) -> Similarity3:
        """
        Create the Similarity3 object that aligns at least two pose pairs. 
        Each pair is of the form (aTi, bTi). Given a list of pairs in frame a, and a list of pairs in frame b, Align() will compute the best-fit Similarity3 aSb transformation to align them. First, the rotation aRb will be computed as the average (Karcher mean) of many estimates aRb (from each pair). Afterwards, the scale factor will be computed using the algorithm described here: http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
        """
    @staticmethod
    def Expmap(v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Similarity3:
        """
        Exponential map at the identity.
        """
    @staticmethod
    def Hat(xi: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        """
        Hat for Similarity3: 
        xi: 7-dim twist (w,u,lambda) where
        Returns: 4*4 element of Lie algebra that can be exponentiated
        """
    @staticmethod
    def Logmap(s: Similarity3) -> numpy.ndarray[tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Log map at the identity $ [R_x,R_y,R_z, t_x, t_y, t_z, \\lambda] $.
        """
    @staticmethod
    def Vee(X: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Vee maps from Lie algebra to tangent vector.
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, s: float) -> None:
        ...
    @typing.overload
    def __init__(self, R: Rot3, t: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], s: float) -> None:
        ...
    @typing.overload
    def __init__(self, R: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], t: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], s: float) -> None:
        ...
    @typing.overload
    def __init__(self, T: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def equals(self, sim: Similarity3, tol: float) -> bool:
        """
        Compare with tolerance.
        """
    def expmap(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Similarity3:
        ...
    def logmap(self, p: Similarity3) -> numpy.ndarray[tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        """
        Calculate 4*4 matrix group equivalent.
        """
    def print(self, s: str = '') -> None:
        """
        Print with optional string.
        """
    def rotation(self) -> Rot3:
        """
        Return a GTSAM rotation.
        """
    def scale(self) -> float:
        """
        Return the scale.
        """
    @typing.overload
    def transformFrom(self, p: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Action on a point p is s*(R*p+t)
        """
    @typing.overload
    def transformFrom(self, T: Pose3) -> Pose3:
        """
        Action on a pose T. 
        |Rs ts| |R t| |Rs*R Rs*t+ts| |0 1/s| * |0 1| = | 0 1/s |, the result is still a Sim3 object. To retrieve a Pose3, we normalized the scale value into 1. |Rs*R Rs*t+ts| |Rs*R s(Rs*t+ts)| | 0 1/s | = | 0 1 | This group action satisfies the compatibility condition. For more details, refer to: https://en.wikipedia.org/wiki/Group_action
        """
    def translation(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return a GTSAM translation.
        """
class SimpleFundamentalMatrix:
    @staticmethod
    def Dim() -> int:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, E: EssentialMatrix, fa: float, fb: float, ca: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], cb: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def dim(self) -> int:
        ...
    def equals(self, other: SimpleFundamentalMatrix, tol: float = 1e-09) -> bool:
        """
        Check equality within a tolerance.
        """
    def localCoordinates(self, F: SimpleFundamentalMatrix) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return local coordinates with respect to another SimpleFundamentalMatrix.
        """
    def matrix(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Return the fundamental matrix representation F = Ka^(-T) * E * Kb^(-1)
        """
    def print(self, s: str = '') -> None:
        ...
    def retract(self, delta: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> SimpleFundamentalMatrix:
        """
        Retract the given vector to get a new SimpleFundamentalMatrix.
        """
class SmartFactorBasePinholeCameraCal3Bundler(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> CameraSetCal3Bundler:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholeCameraCal3DS2(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> CameraSetCal3DS2:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholeCameraCal3Fisheye(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> CameraSetCal3Fisheye:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholeCameraCal3Unified(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> CameraSetCal3Unified:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholeCameraCal3_S2(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> CameraSetCal3_S2:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholePoseCal3Bundler(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> ...:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholePoseCal3DS2(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> ...:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholePoseCal3Fisheye(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> ...:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholePoseCal3Unified(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> ...:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartFactorBasePinholePoseCal3_S2(NonlinearFactor):
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], key: int) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], cameraKeys: list[int]) -> None:
        ...
    def cameras(self, values: Values) -> CameraSetPinholePoseCal3_S2:
        ...
    def dim(self) -> int:
        ...
    def equals(self, p: NonlinearFactor, tol: float = 1e-09) -> bool:
        ...
    def measured(self) -> list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SmartProjectionFactorPinholeCameraCal3Bundler(SmartFactorBasePinholeCameraCal3Bundler):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: CameraSetCal3Bundler, lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: CameraSetCal3Bundler, lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: CameraSetCal3Bundler, lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: CameraSetCal3Bundler) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: CameraSetCal3Bundler, lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: CameraSetCal3Bundler, externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: CameraSetCal3Bundler) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: CameraSetCal3Bundler) -> bool:
        ...
    def triangulateSafe(self, cameras: CameraSetCal3Bundler) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholeCameraCal3DS2(SmartFactorBasePinholeCameraCal3DS2):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: CameraSetCal3DS2, lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: CameraSetCal3DS2, lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: CameraSetCal3DS2, lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: CameraSetCal3DS2) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: CameraSetCal3DS2, lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: CameraSetCal3DS2, externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: CameraSetCal3DS2) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: CameraSetCal3DS2) -> bool:
        ...
    def triangulateSafe(self, cameras: CameraSetCal3DS2) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholeCameraCal3Fisheye(SmartFactorBasePinholeCameraCal3Fisheye):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: CameraSetCal3Fisheye, lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: CameraSetCal3Fisheye, lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: CameraSetCal3Fisheye, lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: CameraSetCal3Fisheye) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: CameraSetCal3Fisheye, lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: CameraSetCal3Fisheye, externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: CameraSetCal3Fisheye) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: CameraSetCal3Fisheye) -> bool:
        ...
    def triangulateSafe(self, cameras: CameraSetCal3Fisheye) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholeCameraCal3Unified(SmartFactorBasePinholeCameraCal3Unified):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: CameraSetCal3Unified, lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: CameraSetCal3Unified, lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: CameraSetCal3Unified, lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: CameraSetCal3Unified) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: CameraSetCal3Unified, lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: CameraSetCal3Unified, externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: CameraSetCal3Unified) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: CameraSetCal3Unified) -> bool:
        ...
    def triangulateSafe(self, cameras: CameraSetCal3Unified) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholeCameraCal3_S2(SmartFactorBasePinholeCameraCal3_S2):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: CameraSetCal3_S2, lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: CameraSetCal3_S2, lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: CameraSetCal3_S2, lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: CameraSetCal3_S2) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: CameraSetCal3_S2, lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: CameraSetCal3_S2, externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: CameraSetCal3_S2) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: CameraSetCal3_S2) -> bool:
        ...
    def triangulateSafe(self, cameras: CameraSetCal3_S2) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholePoseCal3Bundler(SmartFactorBasePinholePoseCal3Bundler):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: ..., lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: ..., lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: ..., lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: ...) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: ..., lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: ..., externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: ...) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: ...) -> bool:
        ...
    def triangulateSafe(self, cameras: ...) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholePoseCal3DS2(SmartFactorBasePinholePoseCal3DS2):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: ..., lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: ..., lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: ..., lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: ...) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: ..., lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: ..., externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: ...) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: ...) -> bool:
        ...
    def triangulateSafe(self, cameras: ...) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholePoseCal3Fisheye(SmartFactorBasePinholePoseCal3Fisheye):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: ..., lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: ..., lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: ..., lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: ...) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: ..., lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: ..., externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: ...) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: ...) -> bool:
        ...
    def triangulateSafe(self, cameras: ...) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholePoseCal3Unified(SmartFactorBasePinholePoseCal3Unified):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: ..., lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: ..., lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: ..., lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: ...) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: ..., lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: ..., externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: ...) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: ...) -> bool:
        ...
    def triangulateSafe(self, cameras: ...) -> TriangulationResult:
        ...
class SmartProjectionFactorPinholePoseCal3_S2(SmartFactorBasePinholePoseCal3_S2):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, params: ... = ...) -> None:
        ...
    def createHessianFactor(self, cameras: CameraSetPinholePoseCal3_S2, lambda: float = 0.0, diagonalDamping: bool = False) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, cameras: CameraSetPinholePoseCal3_S2, lambda: float) -> ...:
        ...
    @typing.overload
    def createJacobianQFactor(self, values: Values, lambda: float) -> ...:
        ...
    def createJacobianSVDFactor(self, cameras: CameraSetPinholePoseCal3_S2, lambda: float) -> JacobianFactor:
        ...
    def decideIfTriangulate(self, cameras: CameraSetPinholePoseCal3_S2) -> bool:
        ...
    def error(self, values: Values) -> float:
        ...
    def isDegenerate(self) -> bool:
        ...
    def isFarPoint(self) -> bool:
        ...
    def isOutlier(self) -> bool:
        ...
    def isPointBehindCamera(self) -> bool:
        ...
    def isValid(self) -> bool:
        ...
    def linearize(self, values: Values) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, cameras: CameraSetPinholePoseCal3_S2, lambda: float = 0.0) -> GaussianFactor:
        ...
    @typing.overload
    def linearizeDamped(self, values: Values, lambda: float = 0.0) -> GaussianFactor:
        ...
    def linearizeToHessian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    def linearizeToJacobian(self, values: Values, lambda: float = 0.0) -> ...:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def reprojectionErrorAfterTriangulation(self, values: Values) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def totalReprojectionError(self, cameras: CameraSetPinholePoseCal3_S2, externalPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> float:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], cameras: CameraSetPinholePoseCal3_S2) -> bool:
        ...
    @typing.overload
    def triangulateAndComputeE(self, E: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], values: Values) -> bool:
        ...
    def triangulateForLinearize(self, cameras: CameraSetPinholePoseCal3_S2) -> bool:
        ...
    def triangulateSafe(self, cameras: CameraSetPinholePoseCal3_S2) -> TriangulationResult:
        ...
class SmartProjectionParams:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, linMode: LinearizationMode = ..., degMode: DegeneracyMode = ..., throwCheirality: bool = False, verboseCheirality: bool = False, retriangulationTh: float = 1e-05) -> None:
        ...
    def __repr__(self, str: str = '') -> str:
        ...
    def print(self, str: str = '') -> None:
        ...
    def setDegeneracyMode(self, degMode: DegeneracyMode) -> None:
        ...
    def setDynamicOutlierRejectionThreshold(self, dynOutRejectionThreshold: bool) -> None:
        ...
    def setEnableEPI(self, enableEPI: bool) -> None:
        ...
    def setLandmarkDistanceThreshold(self, landmarkDistanceThreshold: bool) -> None:
        ...
    def setLinearizationMode(self, linMode: LinearizationMode) -> None:
        ...
    def setRankTolerance(self, rankTol: float) -> None:
        ...
class SmartProjectionPoseFactorCal3Bundler(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Bundler) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Bundler, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Bundler, params: SmartProjectionParams) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Bundler, body_P_sensor: Pose3, params: SmartProjectionParams) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add(self, measured_i: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey_i: int) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def serialize(self) -> str:
        ...
class SmartProjectionPoseFactorCal3DS2(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3DS2) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3DS2, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3DS2, params: SmartProjectionParams) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3DS2, body_P_sensor: Pose3, params: SmartProjectionParams) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add(self, measured_i: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey_i: int) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def serialize(self) -> str:
        ...
class SmartProjectionPoseFactorCal3Fisheye(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Fisheye) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Fisheye, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Fisheye, params: SmartProjectionParams) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Fisheye, body_P_sensor: Pose3, params: SmartProjectionParams) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add(self, measured_i: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey_i: int) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def serialize(self) -> str:
        ...
class SmartProjectionPoseFactorCal3Unified(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Unified) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Unified, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Unified, params: SmartProjectionParams) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3Unified, body_P_sensor: Pose3, params: SmartProjectionParams) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add(self, measured_i: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey_i: int) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def serialize(self) -> str:
        ...
class SmartProjectionPoseFactorCal3_S2(NonlinearFactor):
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3_S2) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3_S2, body_P_sensor: Pose3) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3_S2, params: SmartProjectionParams) -> None:
        ...
    @typing.overload
    def __init__(self, noise: noiseModel.Base, K: Cal3_S2, body_P_sensor: Pose3, params: SmartProjectionParams) -> None:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def add(self, measured_i: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey_i: int) -> None:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    @typing.overload
    def point(self) -> TriangulationResult:
        ...
    @typing.overload
    def point(self, values: Values) -> TriangulationResult:
        ...
    def serialize(self) -> str:
        ...
class SmartProjectionRigFactorPinholePoseCal3Bundler(SmartProjectionFactorPinholePoseCal3Bundler):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, cameraRig: ..., params: ... = ...) -> None:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey: int, cameraId: int = 0) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], poseKeys: list[int], cameraIds: list[int] = []) -> None:
        ...
    def cameraIds(self) -> list[int]:
        ...
    def cameraRig(self) -> ...:
        ...
    def nonUniqueKeys(self) -> list[int]:
        ...
class SmartProjectionRigFactorPinholePoseCal3DS2(SmartProjectionFactorPinholePoseCal3DS2):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, cameraRig: ..., params: ... = ...) -> None:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey: int, cameraId: int = 0) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], poseKeys: list[int], cameraIds: list[int] = []) -> None:
        ...
    def cameraIds(self) -> list[int]:
        ...
    def cameraRig(self) -> ...:
        ...
    def nonUniqueKeys(self) -> list[int]:
        ...
class SmartProjectionRigFactorPinholePoseCal3Fisheye(SmartProjectionFactorPinholePoseCal3Fisheye):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, cameraRig: ..., params: ... = ...) -> None:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey: int, cameraId: int = 0) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], poseKeys: list[int], cameraIds: list[int] = []) -> None:
        ...
    def cameraIds(self) -> list[int]:
        ...
    def cameraRig(self) -> ...:
        ...
    def nonUniqueKeys(self) -> list[int]:
        ...
class SmartProjectionRigFactorPinholePoseCal3Unified(SmartProjectionFactorPinholePoseCal3Unified):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, cameraRig: ..., params: ... = ...) -> None:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey: int, cameraId: int = 0) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], poseKeys: list[int], cameraIds: list[int] = []) -> None:
        ...
    def cameraIds(self) -> list[int]:
        ...
    def cameraRig(self) -> ...:
        ...
    def nonUniqueKeys(self) -> list[int]:
        ...
class SmartProjectionRigFactorPinholePoseCal3_S2(SmartProjectionFactorPinholePoseCal3_S2):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, sharedNoiseModel: noiseModel.Base, cameraRig: CameraSetPinholePoseCal3_S2, params: ... = ...) -> None:
        ...
    @typing.overload
    def add(self, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], poseKey: int, cameraId: int = 0) -> None:
        ...
    @typing.overload
    def add(self, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], poseKeys: list[int], cameraIds: list[int] = []) -> None:
        ...
    def cameraIds(self) -> list[int]:
        ...
    def cameraRig(self) -> CameraSetPinholePoseCal3_S2:
        ...
    def nonUniqueKeys(self) -> list[int]:
        ...
class StereoCamera:
    @staticmethod
    def Dim() -> int:
        """
        Dimensionality of the tangent space.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, pose: Pose3, K: Cal3_S2Stereo) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def backproject(self, z: StereoPoint2) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        back-project a measurement
        """
    def backproject2(self, p: StereoPoint2, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def baseline(self) -> float:
        """
        baseline
        """
    def calibration(self) -> Cal3_S2Stereo:
        """
        Return shared pointer to calibration.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        """
        Dimensionality of the tangent space.
        """
    def equals(self, camera: StereoCamera, tol: float) -> bool:
        """
        equals
        """
    def localCoordinates(self, t2: StereoCamera) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Local coordinates of manifold neighborhood around current value.
        """
    def pose(self) -> Pose3:
        """
        pose
        """
    def print(self, s: str = '') -> None:
        """
        print
        """
    def project(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> StereoPoint2:
        """
        Project 3D point to StereoPoint2 (uL,uR,v)
        """
    def project2(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> StereoPoint2:
        """
        Project 3D point and compute optional derivatives. 
        H1: derivative with respect to pose
        H2: derivative with respect to point
        """
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> StereoCamera:
        """
        Updates a with tangent space delta.
        """
    def serialize(self) -> str:
        ...
class StereoPoint2:
    @staticmethod
    def Expmap(d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> StereoPoint2:
        ...
    @staticmethod
    def Identity() -> StereoPoint2:
        """
        identity
        """
    @staticmethod
    def Logmap(p: StereoPoint2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def __add__(self, arg0: StereoPoint2) -> StereoPoint2:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, uL: float, uR: float, v: float) -> None:
        ...
    @typing.overload
    def __init__(self, v: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __neg__(self) -> StereoPoint2:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def __sub__(self, arg0: StereoPoint2) -> StereoPoint2:
        ...
    def between(self, p2: StereoPoint2) -> StereoPoint2:
        ...
    def compose(self, p1: StereoPoint2) -> StereoPoint2:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, q: StereoPoint2, tol: float) -> bool:
        """
        equals
        """
    def inverse(self) -> StereoPoint2:
        ...
    def localCoordinates(self, t2: StereoPoint2) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '') -> None:
        """
        print
        """
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> StereoPoint2:
        ...
    def serialize(self) -> str:
        ...
    def uL(self) -> float:
        """
        get uL
        """
    def uR(self) -> float:
        """
        get uR
        """
    def v(self) -> float:
        """
        get v
        """
    def vector(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        convert to vector
        """
class SubgraphSolver:
    @typing.overload
    def __init__(self, A: GaussianFactorGraph, parameters: SubgraphSolverParameters, ordering: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, Ab1: GaussianFactorGraph, Ab2: GaussianFactorGraph, parameters: SubgraphSolverParameters, ordering: Ordering) -> None:
        ...
    def optimize(self) -> VectorValues:
        """
        Optimize from zero.
        """
class SubgraphSolverParameters(ConjugateGradientParameters):
    def __init__(self) -> None:
        ...
class Symbol:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, c: str, j: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: int) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def chr(self) -> int:
        """
        Retrieve key character.
        """
    def equals(self, expected: Symbol, tol: float) -> bool:
        """
        Check equality.
        """
    def index(self) -> int:
        """
        Retrieve key index.
        """
    def key(self) -> int:
        """
        return Key (integer) representation
        """
    def print(self, s: str = '') -> None:
        """
        Print.
        """
    def string(self) -> str:
        """
        Return string representation of the key.
        """
class SymbolicBayesNet:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: SymbolicBayesNet) -> None:
        ...
    def __repr__(self, s: str = 'SymbolicBayesNet', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, idx: int) -> SymbolicConditional:
        ...
    def back(self) -> SymbolicConditional:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    def equals(self, bn: SymbolicBayesNet, tol: float) -> bool:
        """
        Check equality.
        """
    def front(self) -> SymbolicConditional:
        ...
    def print(self, s: str = 'SymbolicBayesNet', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def push_back(self, conditional: SymbolicConditional) -> None:
        ...
    @typing.overload
    def push_back(self, bayesNet: SymbolicBayesNet) -> None:
        ...
    @typing.overload
    def saveGraph(self, s: str) -> None:
        ...
    @typing.overload
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def size(self) -> int:
        ...
class SymbolicBayesTree:
    def __getitem__(self, arg0: int) -> SymbolicBayesTreeClique:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: SymbolicBayesTree) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def clear(self) -> None:
        ...
    def deleteCachedShortcuts(self) -> None:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def empty(self) -> bool:
        ...
    def equals(self, other: SymbolicBayesTree, tol: float) -> bool:
        """
        check equality
        """
    def joint(self, key1: int, key2: int) -> SymbolicFactorGraph:
        ...
    def jointBayesNet(self, key1: int, key2: int) -> SymbolicBayesNet:
        ...
    def marginalFactor(self, key: int) -> SymbolicConditional:
        ...
    def numCachedSeparatorMarginals(self) -> int:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def roots(self) -> list[SymbolicBayesTreeClique]:
        ...
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def size(self) -> int:
        ...
class SymbolicBayesTreeClique:
    def __getitem__(self, arg0: int) -> SymbolicBayesTreeClique:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, conditional: SymbolicConditional) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def conditional(self) -> SymbolicConditional:
        ...
    def deleteCachedShortcuts(self) -> None:
        ...
    def equals(self, other: SymbolicBayesTreeClique, tol: float) -> bool:
        ...
    def isRoot(self) -> bool:
        ...
    def nrChildren(self) -> int:
        ...
    def numCachedSeparatorMarginals(self) -> int:
        ...
    def parent(self) -> SymbolicBayesTreeClique:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def treeSize(self) -> int:
        ...
class SymbolicCluster:
    factors: SymbolicFactorGraph
    orderedFrontalKeys: Ordering
    def __getitem__(self, arg0: int) -> SymbolicCluster:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def nrChildren(self) -> int:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SymbolicConditional(SymbolicFactor):
    @staticmethod
    def FromKeys(keys: list[int], nrFrontals: int) -> SymbolicConditional:
        """
        Named constructor from an arbitrary number of keys and frontals.
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: SymbolicConditional) -> None:
        ...
    @typing.overload
    def __init__(self, key: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, parent: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, parent1: int, parent2: int) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, parent1: int, parent2: int, parent3: int) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, c: SymbolicConditional, tol: float) -> bool:
        """
        Check equality.
        """
    def firstFrontalKey(self) -> int:
        ...
    def nrFrontals(self) -> int:
        ...
    def nrParents(self) -> int:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SymbolicEliminationTree:
    @typing.overload
    def __init__(self, factorGraph: SymbolicFactorGraph, structure: VariableIndex, order: Ordering) -> None:
        ...
    @typing.overload
    def __init__(self, factorGraph: SymbolicFactorGraph, order: Ordering) -> None:
        ...
    def __repr__(self, name: str = 'EliminationTree: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, other: SymbolicEliminationTree, tol: float = 1e-09) -> bool:
        """
        Test whether the tree is equal to another.
        """
    def print(self, name: str = 'EliminationTree: ', formatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SymbolicFactor(Factor):
    @staticmethod
    def FromKeys(keys: list[int]) -> SymbolicFactor:
        """
        Constructor from a collection of keys - compatible with boost assign::list_of and boost assign::cref_list_of.
        """
    @typing.overload
    def __init__(self, f: SymbolicFactor) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, j: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, j3: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, j3: int, j4: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, j3: int, j4: int, j5: int) -> None:
        ...
    @typing.overload
    def __init__(self, j1: int, j2: int, j3: int, j4: int, j5: int, j6: int) -> None:
        ...
    def __repr__(self, s: str = 'SymbolicFactor', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, other: SymbolicFactor, tol: float) -> bool:
        ...
    def print(self, s: str = 'SymbolicFactor', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class SymbolicFactorGraph:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, bayesNet: ...) -> None:
        ...
    @typing.overload
    def __init__(self, bayesTree: ...) -> None:
        ...
    def __repr__(self, s: str = 'SymbolicFactorGraph', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def dot(self, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> str:
        ...
    @typing.overload
    def eliminateMultifrontal(self) -> ...:
        ...
    @typing.overload
    def eliminateMultifrontal(self, ordering: Ordering) -> ...:
        ...
    @typing.overload
    def eliminatePartialMultifrontal(self, ordering: Ordering) -> tuple[..., SymbolicFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialMultifrontal(self, keys: list[int]) -> tuple[..., SymbolicFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialSequential(self, ordering: Ordering) -> tuple[..., SymbolicFactorGraph]:
        ...
    @typing.overload
    def eliminatePartialSequential(self, keys: list[int]) -> tuple[..., SymbolicFactorGraph]:
        ...
    @typing.overload
    def eliminateSequential(self) -> ...:
        ...
    @typing.overload
    def eliminateSequential(self, ordering: Ordering) -> ...:
        ...
    def equals(self, fg: SymbolicFactorGraph, tol: float) -> bool:
        ...
    def exists(self, idx: int) -> bool:
        ...
    def keys(self) -> ...:
        ...
    def marginal(self, key_vector: list[int]) -> SymbolicFactorGraph:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, ordering: Ordering) -> ...:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, key_vector: list[int]) -> ...:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, ordering: Ordering, marginalizedVariableOrdering: Ordering) -> ...:
        ...
    @typing.overload
    def marginalMultifrontalBayesNet(self, key_vector: list[int], marginalizedVariableOrdering: Ordering) -> ...:
        ...
    def print(self, s: str = 'SymbolicFactorGraph', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    @typing.overload
    def push_back(self, factor: SymbolicFactor) -> None:
        ...
    @typing.overload
    def push_back(self, graph: SymbolicFactorGraph) -> None:
        ...
    @typing.overload
    def push_back(self, bayesNet: ...) -> None:
        ...
    @typing.overload
    def push_back(self, bayesTree: ...) -> None:
        ...
    @typing.overload
    def push_factor(self, key: int) -> None:
        """
        Push back unary factor.
        """
    @typing.overload
    def push_factor(self, key1: int, key2: int) -> None:
        """
        Push back binary factor.
        """
    @typing.overload
    def push_factor(self, key1: int, key2: int, key3: int) -> None:
        """
        Push back ternary factor.
        """
    @typing.overload
    def push_factor(self, key1: int, key2: int, key3: int, key4: int) -> None:
        """
        Push back 4-way factor.
        """
    def saveGraph(self, s: str, keyFormatter: typing.Callable[[int], str] = ..., writer: DotWriter = ...) -> None:
        ...
    def size(self) -> int:
        ...
class SymbolicJunctionTree:
    def __getitem__(self, arg0: int) -> SymbolicCluster:
        ...
    def __init__(self, eliminationTree: SymbolicEliminationTree) -> None:
        ...
    def __repr__(self, name: str = 'JunctionTree: ', formatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def nrRoots(self) -> int:
        ...
    def print(self, name: str = 'JunctionTree: ', formatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TableDistribution(DiscreteConditional):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, f: TableFactor) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], spec: list[float]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, spec: list[float]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, spec: str) -> None:
        ...
    @typing.overload
    def __init__(self, key: tuple[int, int], spec: str) -> None:
        ...
    def __repr__(self, s: str = 'Table Distribution\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def evaluate(self, values: DiscreteValues) -> float:
        """
        Evaluate the conditional given the values.
        """
    def nrValues(self) -> int:
        """
        Get the number of non-zero values.
        """
    def print(self, s: str = 'Table Distribution\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def table(self) -> TableFactor:
        """
        Return the underlying TableFactor.
        """
class TableFactor(DiscreteFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, potentials: TableFactor) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, table: list[float]) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, spec: str) -> None:
        ...
    @typing.overload
    def __init__(self, keys: DiscreteKeys, dtf: DecisionTreeFactor) -> None:
        ...
    @typing.overload
    def __init__(self, dtf: DecisionTreeFactor) -> None:
        ...
    def __repr__(self, s: str = 'TableFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def error(self, values: DiscreteValues) -> float:
        """
        Calculate error for DiscreteValuesx, is -log(probability).
        """
    def evaluate(self, values: DiscreteValues) -> float:
        """
        Evaluate probability distribution, is just look up in TableFactor.
        """
    def print(self, s: str = 'TableFactor\n', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TransferFactorFundamentalMatrix(NoiseModelFactor):
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
class TransferFactorSimpleFundamentalMatrix(NoiseModelFactor):
    def __init__(self, edge1: EdgeKey, edge2: EdgeKey, triplets: list[tuple[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]], model: noiseModel.Base = None) -> None:
        ...
class TranslationRecovery:
    @typing.overload
    def __init__(self, lmParams: LevenbergMarquardtParams, use_bilinear_translation_factor: bool) -> None:
        ...
    @typing.overload
    def __init__(self, lmParams: LevenbergMarquardtParams) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def addPrior(self, relativeTranslations: list[...], scale: float, betweenTranslations: list[..., 3, 1, 0, 3, ...], graph: NonlinearFactorGraph, priorNoiseModel: noiseModel.Base) -> None:
        """
        Add 3 factors to the graph: 
        A prior on the first point to lie at (0, 0, 0)If betweenTranslations is non-empty, between factors provided by it.If betweenTranslations is empty, a prior on scale of the first relativeTranslations edge. relativeTranslations: unit translation directions between translations to be estimated
        scale: scale for first relative translation which fixes gauge.
        graph: factor graph to which prior is added.
        betweenTranslations: relative translations (with scale) between 2 points in world coordinate frame known a priori.
        priorNoiseModel: the noise model to use with the prior.
        """
    @typing.overload
    def addPrior(self, relativeTranslations: list[...], scale: float, betweenTranslations: list[..., 3, 1, 0, 3, ...], graph: NonlinearFactorGraph) -> None:
        """
        Add 3 factors to the graph: 
        A prior on the first point to lie at (0, 0, 0)If betweenTranslations is non-empty, between factors provided by it.If betweenTranslations is empty, a prior on scale of the first relativeTranslations edge. relativeTranslations: unit translation directions between translations to be estimated
        scale: scale for first relative translation which fixes gauge.
        graph: factor graph to which prior is added.
        betweenTranslations: relative translations (with scale) between 2 points in world coordinate frame known a priori.
        """
    def buildGraph(self, relativeTranslations: list[...]) -> NonlinearFactorGraph:
        """
        Build the factor graph to do the optimization. 
        relativeTranslations: unit translation directions between translations to be estimated
        """
    @typing.overload
    def run(self, relativeTranslations: list[...], scale: float, betweenTranslations: list[..., 3, 1, 0, 3, ...], initialValues: Values) -> Values:
        """
        Build and optimize factor graph. 
        relativeTranslations: the relative translations, in world coordinate frames, vector of BinaryMeasurements of
        scale: scale for first relative translation which fixes gauge. The scale is only used if betweenTranslations is empty.
        betweenTranslations: relative translations (with scale) between 2 points in world coordinate frame known a priori. Unlike relativeTranslations, zero-magnitude betweenTranslations are not treated as hard constraints.
        initialValues: initial values for optimization. Initializes randomly if not provided.
        """
    @typing.overload
    def run(self, relativeTranslations: list[...], scale: float, betweenTranslations: list[..., 3, 1, 0, 3, ...]) -> Values:
        ...
    @typing.overload
    def run(self, relativeTranslations: list[...], scale: float = 1.0) -> Values:
        ...
class TriangulationFactorCal3Bundler(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholeCameraCal3Bundler, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorCal3Bundler, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorCal3DS2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholeCameraCal3DS2, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorCal3DS2, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorCal3Fisheye(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholeCameraCal3Fisheye, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorCal3Fisheye, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorCal3Unified(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholeCameraCal3Unified, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorCal3Unified, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorCal3_S2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholeCameraCal3_S2, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorCal3_S2, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorPoseCal3Bundler(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholePoseCal3Bundler, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorPoseCal3Bundler, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorPoseCal3DS2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholePoseCal3DS2, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorPoseCal3DS2, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorPoseCal3Fisheye(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholePoseCal3Fisheye, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorPoseCal3Fisheye, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorPoseCal3Unified(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholePoseCal3Unified, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorPoseCal3Unified, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationFactorPoseCal3_S2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, camera: PinholePoseCal3_S2, measured: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, pointKey: int, throwCheirality: bool = False, verboseCheirality: bool = False) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def equals(self, p: TriangulationFactorPoseCal3_S2, tol: float = 1e-09) -> bool:
        ...
    def evaluateError(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def measured(self) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
class TriangulationParameters:
    dynamicOutlierRejectionThreshold: float
    enableEPI: bool
    landmarkDistanceThreshold: float
    noiseModel: ...
    rankTolerance: float
    useLOST: bool
    def __init__(self, rankTolerance: float = 1.0, enableEPI: bool = False, landmarkDistanceThreshold: float = -1, dynamicOutlierRejectionThreshold: float = -1, useLOST: bool = False, noiseModel: ... = None) -> None:
        ...
class TriangulationResult:
    class Status:
        """
        Members:
        
          VALID
        
          DEGENERATE
        
          BEHIND_CAMERA
        
          OUTLIER
        
          FAR_POINT
        """
        BEHIND_CAMERA: typing.ClassVar[TriangulationResult.Status]  # value = <Status.BEHIND_CAMERA: 2>
        DEGENERATE: typing.ClassVar[TriangulationResult.Status]  # value = <Status.DEGENERATE: 1>
        FAR_POINT: typing.ClassVar[TriangulationResult.Status]  # value = <Status.FAR_POINT: 4>
        OUTLIER: typing.ClassVar[TriangulationResult.Status]  # value = <Status.OUTLIER: 3>
        VALID: typing.ClassVar[TriangulationResult.Status]  # value = <Status.VALID: 0>
        __members__: typing.ClassVar[dict[str, TriangulationResult.Status]]  # value = {'VALID': <Status.VALID: 0>, 'DEGENERATE': <Status.DEGENERATE: 1>, 'BEHIND_CAMERA': <Status.BEHIND_CAMERA: 2>, 'OUTLIER': <Status.OUTLIER: 3>, 'FAR_POINT': <Status.FAR_POINT: 4>}
        def __and__(self, other: typing.Any) -> typing.Any:
            ...
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __ge__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __gt__(self, other: typing.Any) -> bool:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __invert__(self) -> typing.Any:
            ...
        def __le__(self, other: typing.Any) -> bool:
            ...
        def __lt__(self, other: typing.Any) -> bool:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __or__(self, other: typing.Any) -> typing.Any:
            ...
        def __rand__(self, other: typing.Any) -> typing.Any:
            ...
        def __repr__(self) -> str:
            ...
        def __ror__(self, other: typing.Any) -> typing.Any:
            ...
        def __rxor__(self, other: typing.Any) -> typing.Any:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        def __xor__(self, other: typing.Any) -> typing.Any:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    status: ...
    @staticmethod
    def BehindCamera() -> TriangulationResult:
        ...
    @staticmethod
    def Degenerate() -> TriangulationResult:
        ...
    @staticmethod
    def FarPoint() -> TriangulationResult:
        ...
    @staticmethod
    def Outlier() -> TriangulationResult:
        ...
    def __init__(self, p: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def behindCamera(self) -> bool:
        ...
    def degenerate(self) -> bool:
        ...
    def farPoint(self) -> bool:
        ...
    def get(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def outlier(self) -> bool:
        ...
    def valid(self) -> bool:
        ...
class Unit3:
    @staticmethod
    def Dim() -> int:
        """
        Dimensionality of tangent space = 2 DOF.
        """
    @typing.overload
    def FromPoint3(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Unit3:
        """
        Named constructor from Point3 with optional Jacobian.
        """
    @typing.overload
    def FromPoint3(self, point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> Unit3:
        """
        Named constructor from Point3 with optional Jacobian.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, pose: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None:
        ...
    @typing.overload
    def __init__(self, p: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], f: float) -> None:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    @typing.overload
    def basis(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[2]], numpy.dtype[numpy.float64]]:
        """
        Returns the local coordinate frame to tangent plane It is a 3*2 matrix [b1 b2] composed of two orthogonal directions tangent to the sphere at the current direction. 
        Provides derivatives of the basis with the two basis vectors stacked up as a 6x1.
        """
    @typing.overload
    def basis(self, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[2]], numpy.dtype[numpy.float64]]:
        """
        Returns the local coordinate frame to tangent plane It is a 3*2 matrix [b1 b2] composed of two orthogonal directions tangent to the sphere at the current direction. 
        Provides derivatives of the basis with the two basis vectors stacked up as a 6x1.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        """
        Dimensionality of tangent space = 2 DOF.
        """
    @typing.overload
    def dot(self, q: Unit3) -> float:
        """
        Return dot product with q.
        """
    @typing.overload
    def dot(self, q: Unit3, H1: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H2: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> float:
        """
        Return dot product with q.
        """
    @typing.overload
    def equals(self, s: Unit3, tol: float) -> bool:
        """
        The equals function with tolerance.
        """
    @typing.overload
    def equals(self, expected: Unit3, tol: float) -> bool:
        ...
    @typing.overload
    def errorVector(self, q: Unit3) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Signed, vector-valued error between two directions NOTE(hayk): This method has zero derivatives if this (p) and q are orthogonal.
        """
    @typing.overload
    def errorVector(self, q: Unit3, H_p: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], H_q: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Signed, vector-valued error between two directions NOTE(hayk): This method has zero derivatives if this (p) and q are orthogonal.
        """
    def localCoordinates(self, s: Unit3) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        The local coordinates function.
        """
    @typing.overload
    def point3(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return unit-norm Point3.
        """
    @typing.overload
    def point3(self, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return unit-norm Point3.
        """
    def print(self, s: str = '') -> None:
        """
        The print fuction.
        """
    def retract(self, v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> Unit3:
        """
        The retract function.
        """
    def serialize(self) -> str:
        ...
    def skew(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]]:
        """
        Return skew-symmetric associated with 3D point on unit sphere.
        """
    @typing.overload
    def unitVector(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return unit-norm Vector.
        """
    @typing.overload
    def unitVector(self, H: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Return unit-norm Vector.
        """
class Value:
    def __repr__(self, str: str = '') -> str:
        ...
    def dim(self) -> int:
        """
        Return the dimensionality of the tangent space of this value. 
        This is the dimensionality of delta passed into retract() and of the vector returned by localCoordinates(). The dimensionality of the tangent space  Returns: The dimensionality of the tangent space
        """
    def print(self, str: str = '') -> None:
        """
        Print this value, for debugging and unit tests.
        """
class Values:
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: Values) -> None:
        ...
    def __repr__(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def atCal3Bundler(self, j: int) -> Cal3Bundler:
        ...
    def atCal3DS2(self, j: int) -> Cal3DS2:
        ...
    def atCal3Fisheye(self, j: int) -> Cal3Fisheye:
        ...
    def atCal3Unified(self, j: int) -> Cal3Unified:
        ...
    def atCal3_S2(self, j: int) -> Cal3_S2:
        ...
    def atCal3f(self, j: int) -> Cal3f:
        ...
    def atConstantBias(self, j: int) -> ...:
        ...
    def atDouble(self, j: int) -> float:
        ...
    def atEssentialMatrix(self, j: int) -> EssentialMatrix:
        ...
    def atFundamentalMatrix(self, j: int) -> FundamentalMatrix:
        ...
    def atMatrix(self, j: int) -> numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]:
        ...
    def atNavState(self, j: int) -> ...:
        ...
    def atOrientedPlane3(self, j: int) -> OrientedPlane3:
        ...
    def atPinholeCameraCal3Bundler(self, j: int) -> PinholeCameraCal3Bundler:
        ...
    def atPinholeCameraCal3DS2(self, j: int) -> PinholeCameraCal3DS2:
        ...
    def atPinholeCameraCal3Fisheye(self, j: int) -> PinholeCameraCal3Fisheye:
        ...
    def atPinholeCameraCal3Unified(self, j: int) -> PinholeCameraCal3Unified:
        ...
    def atPinholeCameraCal3_S2(self, j: int) -> PinholeCameraCal3_S2:
        ...
    def atPinholeCameraCal3f(self, j: int) -> PinholeCameraCal3f:
        ...
    def atPinholePoseCal3Bundler(self, j: int) -> PinholePoseCal3Bundler:
        ...
    def atPinholePoseCal3DS2(self, j: int) -> PinholePoseCal3DS2:
        ...
    def atPinholePoseCal3Fisheye(self, j: int) -> PinholePoseCal3Fisheye:
        ...
    def atPinholePoseCal3Unified(self, j: int) -> PinholePoseCal3Unified:
        ...
    def atPinholePoseCal3_S2(self, j: int) -> PinholePoseCal3_S2:
        ...
    def atPinholePoseCal3f(self, j: int) -> ...:
        ...
    def atPoint2(self, j: int) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def atPoint3(self, j: int) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def atPose2(self, j: int) -> Pose2:
        ...
    def atPose3(self, j: int) -> Pose3:
        ...
    def atRot2(self, j: int) -> Rot2:
        ...
    def atRot3(self, j: int) -> Rot3:
        ...
    def atSO3(self, j: int) -> SO3:
        ...
    def atSO4(self, j: int) -> SO4:
        ...
    def atSOn(self, j: int) -> SOn:
        ...
    def atSimilarity2(self, j: int) -> Similarity2:
        ...
    def atSimilarity3(self, j: int) -> Similarity3:
        ...
    def atSimpleFundamentalMatrix(self, j: int) -> SimpleFundamentalMatrix:
        ...
    def atUnit3(self, j: int) -> Unit3:
        ...
    def atVector(self, j: int) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def clear(self) -> None:
        """
        Remove all variables from the config.
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self) -> int:
        """
        Compute the total dimensionality of all values ( $ O(n) $)
        """
    def empty(self) -> bool:
        """
        whether the config is empty
        """
    def equals(self, other: Values, tol: float) -> bool:
        """
        Test whether the sets of keys and values are identical.
        """
    def erase(self, j: int) -> None:
        """
        Remove a variable from the config, throws KeyDoesNotExist<J> if j is not present.
        """
    def exists(self, j: int) -> bool:
        """
        Check if a value exists with key j. 
        See exists<>(Key j) and exists(const TypedKey& j) for versions that return the value if it exists.
        """
    @typing.overload
    def insert(self, values: Values) -> None:
        """
        Add a set of variables, throws KeyAlreadyExists<J> if a key is already present.
        """
    @typing.overload
    def insert(self, j: int, vector: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert(self, j: int, matrix: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert(self, j: int, point2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert(self, j: int, point3: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert(self, j: int, rot2: Rot2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, pose2: Pose2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, R: SO3) -> None:
        ...
    @typing.overload
    def insert(self, j: int, Q: SO4) -> None:
        ...
    @typing.overload
    def insert(self, j: int, P: SOn) -> None:
        ...
    @typing.overload
    def insert(self, j: int, rot3: Rot3) -> None:
        ...
    @typing.overload
    def insert(self, j: int, pose3: Pose3) -> None:
        ...
    @typing.overload
    def insert(self, j: int, similarity2: Similarity2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, similarity3: Similarity3) -> None:
        ...
    @typing.overload
    def insert(self, j: int, unit3: Unit3) -> None:
        ...
    @typing.overload
    def insert(self, j: int, cal3bundler: Cal3Bundler) -> None:
        ...
    @typing.overload
    def insert(self, j: int, cal3f: Cal3f) -> None:
        ...
    @typing.overload
    def insert(self, j: int, cal3_s2: Cal3_S2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, cal3ds2: Cal3DS2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, cal3fisheye: Cal3Fisheye) -> None:
        ...
    @typing.overload
    def insert(self, j: int, cal3unified: Cal3Unified) -> None:
        ...
    @typing.overload
    def insert(self, j: int, E: EssentialMatrix) -> None:
        ...
    @typing.overload
    def insert(self, j: int, F: FundamentalMatrix) -> None:
        ...
    @typing.overload
    def insert(self, j: int, F: SimpleFundamentalMatrix) -> None:
        ...
    @typing.overload
    def insert(self, j: int, plane: OrientedPlane3) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholeCameraCal3Bundler) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholeCameraCal3f) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholeCameraCal3_S2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholeCameraCal3DS2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholeCameraCal3Fisheye) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholeCameraCal3Unified) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholePoseCal3Bundler) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: ...) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholePoseCal3_S2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholePoseCal3DS2) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholePoseCal3Fisheye) -> None:
        ...
    @typing.overload
    def insert(self, j: int, camera: PinholePoseCal3Unified) -> None:
        ...
    @typing.overload
    def insert(self, j: int, constant_bias: ...) -> None:
        ...
    @typing.overload
    def insert(self, j: int, nav_state: ...) -> None:
        ...
    @typing.overload
    def insert(self, j: int, c: float) -> None:
        ...
    def insertPoint2(self, j: int, val: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def insertPoint3(self, j: int, val: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def insert_E(self, j: int, E: EssentialMatrix) -> None:
        ...
    @typing.overload
    def insert_F(self, j: int, F: FundamentalMatrix) -> None:
        ...
    @typing.overload
    def insert_F(self, j: int, F: SimpleFundamentalMatrix) -> None:
        ...
    def insert_P(self, j: int, P: SOn) -> None:
        ...
    def insert_Q(self, j: int, Q: SO4) -> None:
        ...
    def insert_R(self, j: int, R: SO3) -> None:
        ...
    def insert_c(self, j: int, c: float) -> None:
        ...
    def insert_cal3_s2(self, j: int, cal3_s2: Cal3_S2) -> None:
        ...
    def insert_cal3bundler(self, j: int, cal3bundler: Cal3Bundler) -> None:
        ...
    def insert_cal3ds2(self, j: int, cal3ds2: Cal3DS2) -> None:
        ...
    def insert_cal3f(self, j: int, cal3f: Cal3f) -> None:
        ...
    def insert_cal3fisheye(self, j: int, cal3fisheye: Cal3Fisheye) -> None:
        ...
    def insert_cal3unified(self, j: int, cal3unified: Cal3Unified) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholeCameraCal3Bundler) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholeCameraCal3f) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholeCameraCal3_S2) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholeCameraCal3DS2) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholeCameraCal3Fisheye) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholeCameraCal3Unified) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholePoseCal3Bundler) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: ...) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholePoseCal3_S2) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholePoseCal3DS2) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholePoseCal3Fisheye) -> None:
        ...
    @typing.overload
    def insert_camera(self, j: int, camera: PinholePoseCal3Unified) -> None:
        ...
    def insert_constant_bias(self, j: int, constant_bias: ...) -> None:
        ...
    def insert_matrix(self, j: int, matrix: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def insert_nav_state(self, j: int, nav_state: ...) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, values: Values) -> None:
        """
        Update a set of variables. 
        If any variable key does not exist, then perform an insert.
        """
    @typing.overload
    def insert_or_assign(self, j: int, vector: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, matrix: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, point2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, point3: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, rot2: Rot2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, pose2: Pose2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, R: SO3) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, Q: SO4) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, P: SOn) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, rot3: Rot3) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, pose3: Pose3) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, similarity2: Similarity2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, similarity3: Similarity3) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, unit3: Unit3) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, cal3bundler: Cal3Bundler) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, cal3f: Cal3f) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, cal3_s2: Cal3_S2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, cal3ds2: Cal3DS2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, cal3fisheye: Cal3Fisheye) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, cal3unified: Cal3Unified) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, E: EssentialMatrix) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, F: FundamentalMatrix) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, F: SimpleFundamentalMatrix) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, plane: OrientedPlane3) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholeCameraCal3Bundler) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholeCameraCal3f) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholeCameraCal3_S2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholeCameraCal3DS2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholeCameraCal3Fisheye) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholeCameraCal3Unified) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholePoseCal3Bundler) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: ...) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholePoseCal3_S2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholePoseCal3DS2) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholePoseCal3Fisheye) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, camera: PinholePoseCal3Unified) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, constant_bias: ...) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, nav_state: ...) -> None:
        ...
    @typing.overload
    def insert_or_assign(self, j: int, c: float) -> None:
        ...
    def insert_plane(self, j: int, plane: OrientedPlane3) -> None:
        ...
    def insert_point2(self, j: int, point2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def insert_point3(self, j: int, point3: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def insert_pose2(self, j: int, pose2: Pose2) -> None:
        ...
    def insert_pose3(self, j: int, pose3: Pose3) -> None:
        ...
    def insert_rot2(self, j: int, rot2: Rot2) -> None:
        ...
    def insert_rot3(self, j: int, rot3: Rot3) -> None:
        ...
    def insert_similarity2(self, j: int, similarity2: Similarity2) -> None:
        ...
    def insert_similarity3(self, j: int, similarity3: Similarity3) -> None:
        ...
    def insert_unit3(self, j: int, unit3: Unit3) -> None:
        ...
    def insert_vector(self, j: int, vector: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def keys(self) -> list[int]:
        """
        Returns a vector of keys in the config. 
        Note: by construction, the list is ordered
        """
    def localCoordinates(self, cp: Values) -> VectorValues:
        """
        Get a delta config about a linearization point c0 (*this)
        """
    def print(self, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def retract(self, delta: VectorValues) -> Values:
        """
        Add a delta config to current config and returns a new config.
        """
    def serialize(self) -> str:
        ...
    def size(self) -> int:
        """
        The number of variables in this config.
        """
    def swap(self, other: Values) -> None:
        """
        Swap the contents of two Values without copying data.
        """
    @typing.overload
    def update(self, values: Values) -> None:
        """
        update the current available values without adding new ones
        """
    @typing.overload
    def update(self, j: int, vector: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def update(self, j: int, matrix: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def update(self, j: int, point2: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def update(self, j: int, point3: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def update(self, j: int, rot2: Rot2) -> None:
        ...
    @typing.overload
    def update(self, j: int, pose2: Pose2) -> None:
        ...
    @typing.overload
    def update(self, j: int, R: SO3) -> None:
        ...
    @typing.overload
    def update(self, j: int, Q: SO4) -> None:
        ...
    @typing.overload
    def update(self, j: int, P: SOn) -> None:
        ...
    @typing.overload
    def update(self, j: int, rot3: Rot3) -> None:
        ...
    @typing.overload
    def update(self, j: int, pose3: Pose3) -> None:
        ...
    @typing.overload
    def update(self, j: int, similarity2: Similarity2) -> None:
        ...
    @typing.overload
    def update(self, j: int, similarity3: Similarity3) -> None:
        ...
    @typing.overload
    def update(self, j: int, unit3: Unit3) -> None:
        ...
    @typing.overload
    def update(self, j: int, cal3bundler: Cal3Bundler) -> None:
        ...
    @typing.overload
    def update(self, j: int, cal3f: Cal3f) -> None:
        ...
    @typing.overload
    def update(self, j: int, cal3_s2: Cal3_S2) -> None:
        ...
    @typing.overload
    def update(self, j: int, cal3ds2: Cal3DS2) -> None:
        ...
    @typing.overload
    def update(self, j: int, cal3fisheye: Cal3Fisheye) -> None:
        ...
    @typing.overload
    def update(self, j: int, cal3unified: Cal3Unified) -> None:
        ...
    @typing.overload
    def update(self, j: int, E: EssentialMatrix) -> None:
        ...
    @typing.overload
    def update(self, j: int, F: FundamentalMatrix) -> None:
        ...
    @typing.overload
    def update(self, j: int, F: SimpleFundamentalMatrix) -> None:
        ...
    @typing.overload
    def update(self, j: int, plane: OrientedPlane3) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholeCameraCal3Bundler) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholeCameraCal3f) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholeCameraCal3_S2) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholeCameraCal3DS2) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholeCameraCal3Fisheye) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholeCameraCal3Unified) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholePoseCal3Bundler) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: ...) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholePoseCal3_S2) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholePoseCal3DS2) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholePoseCal3Fisheye) -> None:
        ...
    @typing.overload
    def update(self, j: int, camera: PinholePoseCal3Unified) -> None:
        ...
    @typing.overload
    def update(self, j: int, constant_bias: ...) -> None:
        ...
    @typing.overload
    def update(self, j: int, nav_state: ...) -> None:
        ...
    @typing.overload
    def update(self, j: int, c: float) -> None:
        ...
    def zeroVectors(self) -> VectorValues:
        """
        Return a VectorValues of zero vectors for each variable in this Values.
        """
class VariableIndex:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, factorGraph: ...) -> None:
        ...
    @typing.overload
    def __init__(self, factorGraph: ...) -> None:
        ...
    @typing.overload
    def __init__(self, factorGraph: ...) -> None:
        ...
    @typing.overload
    def __init__(self, other: VariableIndex) -> None:
        ...
    def __repr__(self, s: str = 'VariableIndex: ', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def at(self, variable: int) -> list[int]:
        """
        Access a list of factors by variable.
        """
    def empty(self, variable: int) -> bool:
        """
        Return true if no factors associated with a variable.
        """
    def equals(self, other: VariableIndex, tol: float) -> bool:
        """
        Test for equality (for unit tests and debug assertions).
        """
    def nEntries(self) -> int:
        """
        The number of nonzero blocks, i.e. the number of variable-factor entries.
        """
    def nFactors(self) -> int:
        """
        The number of factors in the original factor graph.
        """
    def print(self, s: str = 'VariableIndex: ', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def size(self) -> int:
        """
        The number of variable entries. This is equal to the number of unique variable Keys.
        """
class VectorComponentFactorChebyshev1Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class VectorComponentFactorChebyshev2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class VectorComponentFactorChebyshev2Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class VectorComponentFactorFourierBasis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: float, model: noiseModel.Base, M: int, N: int, i: int, x: float, a: float, b: float) -> None:
        ...
class VectorDerivativeFactorChebyshev1Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorDerivativeFactorChebyshev2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorDerivativeFactorChebyshev2Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorDerivativeFactorFourierBasis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorEvaluationFactorChebyshev1Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorEvaluationFactorChebyshev2(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorEvaluationFactorChebyshev2Basis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorEvaluationFactorFourierBasis(NoiseModelFactor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float) -> None:
        ...
    @typing.overload
    def __init__(self, key: int, z: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], model: noiseModel.Base, M: int, N: int, x: float, a: float, b: float) -> None:
        ...
class VectorValues:
    @staticmethod
    def Zero(other: VectorValues) -> VectorValues:
        """
        Create a VectorValues with the same structure as other, but filled with zeros.
        """
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, other: VectorValues) -> None:
        ...
    @typing.overload
    def __init__(self, first: VectorValues, second: VectorValues) -> None:
        ...
    def __repr__(self, s: str = 'VectorValues', keyFormatter: typing.Callable[[int], str] = ...) -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def _repr_html_(self) -> str:
        """
        Output as a html table.
        """
    def add(self, c: VectorValues) -> VectorValues:
        """
        Element-wise addition, synonym for operator+(). 
        Both VectorValues must have the same structure (checked when NDEBUG is not defined).
        """
    def addInPlace(self, c: VectorValues) -> None:
        """
        Element-wise addition in-place, synonym for operator+=(). 
        Both VectorValues must have the same structure (checked when NDEBUG is not defined).
        """
    def at(self, j: int) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Read/write access to the vector value with key j, throws std::out_of_range if j does not exist, identical to operator[](Key).
        """
    def deserialize(self, serialized: str) -> None:
        ...
    def dim(self, j: int) -> int:
        """
        Return the dimension of variable j.
        """
    def dot(self, v: VectorValues) -> float:
        """
        Dot product with another VectorValues, interpreting both as vectors of their concatenated values. 
        Both VectorValues must have the same structure (checked when NDEBUG is not defined).
        """
    def equals(self, x: VectorValues, tol: float) -> bool:
        """
        equals required by Testable for unit testing
        """
    def exists(self, j: int) -> bool:
        """
        Check whether a variable with key j exists.
        """
    def hasSameStructure(self, other: VectorValues) -> bool:
        """
        Check if this VectorValues has the same structure (keys and dimensions) as another.
        """
    @typing.overload
    def insert(self, j: int, value: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        """
        Insert a vector value with key j. 
        value: The vector to be inserted.
        j: The index with which the value will be associated.
        """
    @typing.overload
    def insert(self, values: VectorValues) -> None:
        """
        Insert all values from values. 
        Throws an invalid_argument exception if any keys to be inserted are already used.
        """
    def norm(self) -> float:
        """
        Vector L2 norm.
        """
    def print(self, s: str = 'VectorValues', keyFormatter: typing.Callable[[int], str] = ...) -> None:
        ...
    def scale(self, a: float) -> VectorValues:
        """
        Element-wise scaling by a constant.
        """
    def scaleInPlace(self, alpha: float) -> None:
        """
        Element-wise scaling by a constant in-place.
        """
    def serialize(self) -> str:
        ...
    def setZero(self) -> None:
        """
        Set all values to zero vectors.
        """
    def size(self) -> int:
        """
        Number of variables stored.
        """
    def squaredNorm(self) -> float:
        """
        Squared vector L2 norm.
        """
    def subtract(self, c: VectorValues) -> VectorValues:
        """
        Element-wise subtraction, synonym for operator-(). 
        Both VectorValues must have the same structure (checked when NDEBUG is not defined).
        """
    def update(self, values: VectorValues) -> None:
        """
        For all key/value pairs in values, replace values with corresponding keys in this class with those in values. 
        Throws std::out_of_range if any keys in values are not present in this class.
        """
    @typing.overload
    def vector(self) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Retrieve the entire solution as a single vector.
        """
    @typing.overload
    def vector(self, keys: list[int]) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Access a vector that is a subset of relevant keys.
        """
def ConvertNoiseModel(model: noiseModel.Base, d: int) -> noiseModel.Base:
    ...
def EliminateDiscrete(factors: DiscreteFactorGraph, frontalKeys: Ordering) -> tuple[DiscreteConditional, DiscreteFactor]:
    ...
def EliminateForMPE(factors: DiscreteFactorGraph, frontalKeys: Ordering) -> tuple[DiscreteConditional, DiscreteFactor]:
    ...
def EliminateQR(factors: GaussianFactorGraph, keys: Ordering) -> tuple[GaussianConditional, JacobianFactor]:
    ...
def EpipolarTransfer(Fca: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]], pa: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]], Fcb: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]], pb: numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
def FindKarcherMeanPoint2(elements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]) -> numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
def FindKarcherMeanPoint3(elements: list[numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
def FindKarcherMeanPose2(elements: list[Pose2]) -> Pose2:
    ...
def FindKarcherMeanPose3(elements: list[Pose3]) -> Pose3:
    ...
def FindKarcherMeanRot2(elements: list[Rot2]) -> Rot2:
    ...
def FindKarcherMeanRot3(elements: list[Rot3]) -> Rot3:
    ...
def FindKarcherMeanSO3(elements: list[SO3]) -> SO3:
    ...
def FindKarcherMeanSO4(elements: list[SO4]) -> SO4:
    ...
def IndexPairSetAsArray(set: set[IndexPair]) -> list[IndexPair]:
    ...
def PrintDiscreteValues(values: DiscreteValues, s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
    ...
def PrintKeyList(keys: ..., s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
    ...
def PrintKeySet(keys: ..., s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
    ...
def PrintKeyVector(keys: list[int], s: str = '', keyFormatter: typing.Callable[[int], str] = ...) -> None:
    ...
def cartesianProduct(keys: DiscreteKeys) -> list[DiscreteValues]:
    ...
@typing.overload
def checkConvergence(relativeErrorTreshold: float, absoluteErrorTreshold: float, errorThreshold: float, currentError: float, newError: float) -> bool:
    ...
@typing.overload
def checkConvergence(params: NonlinearOptimizerParams, currentError: float, newError: float) -> bool:
    ...
@typing.overload
def html(values: DiscreteValues, keyFormatter: typing.Callable[[int], str] = ...) -> str:
    ...
@typing.overload
def html(values: DiscreteValues, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
    ...
def initialCamerasAndPointsEstimate(db: SfmData) -> Values:
    ...
def initialCamerasEstimate(db: SfmData) -> Values:
    ...
def isDebugVersion() -> bool:
    ...
def linear_independent(A: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], B: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], tol: float) -> bool:
    ...
def load2D(filename: str, model: noiseModel.Diagonal = None, maxIndex: int = 0, addNoise: bool = False, smart: bool = True, noiseFormat: NoiseFormat = NoiseFormat.NoiseFormatAUTO, kernelFunctionType: KernelFunctionType = KernelFunctionType.KernelFunctionTypeNONE) -> tuple[NonlinearFactorGraph, Values]:
    ...
def load3D(filename: str) -> tuple[NonlinearFactorGraph, Values]:
    ...
@typing.overload
def markdown(values: DiscreteValues, keyFormatter: typing.Callable[[int], str] = ...) -> str:
    ...
@typing.overload
def markdown(values: DiscreteValues, keyFormatter: typing.Callable[[int], str], names: dict[int, list[str]]) -> str:
    ...
def mrsymbol(c: int, label: int, j: int) -> int:
    ...
def mrsymbolChr(key: int) -> int:
    ...
def mrsymbolIndex(key: int) -> int:
    ...
def mrsymbolLabel(key: int) -> int:
    ...
def parse2DFactors(filename: str) -> list[BetweenFactorPose2]:
    ...
def parse3DFactors(filename: str) -> list[BetweenFactorPose3]:
    ...
def readBal(filename: str) -> SfmData:
    ...
def readG2o(filename: str, is3D: bool = False, kernelFunctionType: KernelFunctionType = KernelFunctionType.KernelFunctionTypeNONE) -> tuple[NonlinearFactorGraph, Values]:
    ...
def save2D(graph: NonlinearFactorGraph, config: Values, model: noiseModel.Diagonal, filename: str) -> None:
    ...
def symbol(chr: str, index: int) -> int:
    ...
def symbolChr(key: int) -> int:
    ...
def symbolIndex(key: int) -> int:
    ...
@typing.overload
def triangulateNonlinear(poses: list[Pose3], sharedCal: Cal3_S2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(cameras: CameraSetCal3_S2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(poses: list[Pose3], sharedCal: Cal3DS2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(cameras: CameraSetCal3DS2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(poses: list[Pose3], sharedCal: Cal3Bundler, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(cameras: CameraSetCal3Bundler, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(poses: list[Pose3], sharedCal: Cal3Fisheye, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(cameras: CameraSetCal3Fisheye, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(poses: list[Pose3], sharedCal: Cal3Unified, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateNonlinear(cameras: CameraSetCal3Unified, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], initialEstimate: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(poses: list[Pose3], sharedCal: Cal3_S2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(cameras: CameraSetCal3_S2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None, useLOST: bool = False) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(poses: list[Pose3], sharedCal: Cal3DS2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(cameras: CameraSetCal3DS2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None, useLOST: bool = False) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(poses: list[Pose3], sharedCal: Cal3Bundler, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(cameras: CameraSetCal3Bundler, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None, useLOST: bool = False) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(poses: list[Pose3], sharedCal: Cal3Fisheye, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(cameras: CameraSetCal3Fisheye, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None, useLOST: bool = False) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(poses: list[Pose3], sharedCal: Cal3Unified, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulatePoint3(cameras: CameraSetCal3Unified, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], rank_tol: float, optimize: bool, model: ... = None, useLOST: bool = False) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
    ...
@typing.overload
def triangulateSafe(cameras: CameraSetCal3_S2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], params: TriangulationParameters) -> TriangulationResult:
    ...
@typing.overload
def triangulateSafe(cameras: CameraSetCal3DS2, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], params: TriangulationParameters) -> TriangulationResult:
    ...
@typing.overload
def triangulateSafe(cameras: CameraSetCal3Bundler, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], params: TriangulationParameters) -> TriangulationResult:
    ...
@typing.overload
def triangulateSafe(cameras: CameraSetCal3Fisheye, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], params: TriangulationParameters) -> TriangulationResult:
    ...
@typing.overload
def triangulateSafe(cameras: CameraSetCal3Unified, measurements: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]], params: TriangulationParameters) -> TriangulationResult:
    ...
def writeBAL(filename: str, data: SfmData) -> bool:
    ...
def writeG2o(graph: NonlinearFactorGraph, estimate: Values, filename: str) -> None:
    ...
