#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <string>
#include <vector>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "deformable_manipulation_experiment_params/task_enums.h"

namespace smmap
{
    ////////////////////////////////////////////////////////////////////////////
    // Visualization Settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetDisableSmmapVisualizations(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetVisualizeObjectDesiredMotion(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetVisualizeGripperMotion(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetVisualizeObjectPredictedMotion(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetVisualizeRRT(const std::shared_ptr<rclcpp::Node>& nh, const bool default_vis = true);
    bool GetVisualizeFreeSpaceGraph(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetVisualizeCorrespondences(const std::shared_ptr<rclcpp::Node>& nh);
    bool VisualizeStrainLines(const std::shared_ptr<rclcpp::Node>& nh);

    int GetViewerWidth(const std::shared_ptr<rclcpp::Node>& nh);     // Pixels
    int GetViewerHeight(const std::shared_ptr<rclcpp::Node>& nh);    // Pixels

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    std::string GetTestId(const std::shared_ptr<rclcpp::Node>& nh);
    DeformableType GetDeformableType(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetTaskTypeString(const std::shared_ptr<rclcpp::Node>& nh);
    TaskType GetTaskType(const std::shared_ptr<rclcpp::Node>& nh);
    double GetMaxTime(const std::shared_ptr<rclcpp::Node>& nh);
    double GetMaxStretchFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetMaxBandLength(const std::shared_ptr<rclcpp::Node>& nh);
    float GetMaxStrain(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Error calculation settings
    ////////////////////////////////////////////////////////////////////////////

    double GetErrorThresholdAlongNormal(const std::shared_ptr<rclcpp::Node>& nh);
    double GetErrorThresholdDistanceToNormal(const std::shared_ptr<rclcpp::Node>& nh);
    double GetErrorThresholdTaskDone(const std::shared_ptr<rclcpp::Node>& nh);
    double GetDesiredMotionScalingFactor(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Gripper Size Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetGripperApperture(const std::shared_ptr<rclcpp::Node>& nh);                     // METERS
    // TODO: where is this still used? Is it being used correctly vs ControllerMinDistToObstacles?
    double GetRobotGripperRadius();                                     // METERS
    // Used by the "older" avoidance code, I.e. LeastSquaresControllerWithObjectAvoidance
    double GetRobotMinGripperDistanceToObstacles();                     // METERS
    double GetControllerMinDistanceToObstacles(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    double GetRRTMinGripperDistanceToObstacles(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    double GetRRTTargetMinDistanceScaleFactor(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Table Size Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetTableSurfaceX(const std::shared_ptr<rclcpp::Node>& nh);        // METERS
    float GetTableSurfaceY(const std::shared_ptr<rclcpp::Node>& nh);        // METERS
    float GetTableSurfaceZ(const std::shared_ptr<rclcpp::Node>& nh);        // METERS
    float GetTableHalfExtentsX(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetTableHalfExtentsY(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetTableHeight(const std::shared_ptr<rclcpp::Node>& nh);          // METERS
    float GetTableLegWidth(const std::shared_ptr<rclcpp::Node>& nh);        // METERS
    float GetTableThickness(const std::shared_ptr<rclcpp::Node>& nh);       // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Cylinder Size Settings
    // TODO: Update launch files to contain these defaults
    ////////////////////////////////////////////////////////////////////////////

    float GetCylinderRadius(const std::shared_ptr<rclcpp::Node>& nh);           // METERS
    float GetCylinderHeight(const std::shared_ptr<rclcpp::Node>& nh);           // METERS
    float GetCylinderCenterOfMassX(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetCylinderCenterOfMassY(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetCylinderCenterOfMassZ(const std::shared_ptr<rclcpp::Node>& nh);    // METERS

    // Cylinder Size settings for WAFR task case
    float GetWafrCylinderRadius(const std::shared_ptr<rclcpp::Node>& nh);       // METERS
    float GetWafrCylinderHeight(const std::shared_ptr<rclcpp::Node>& nh);       // METERS
    float GetWafrCylinderRelativeCenterOfMassX(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetWafrCylinderRelativeCenterOfMassY(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetWafrCylinderRelativeCenterOfMassZ(const std::shared_ptr<rclcpp::Node>& nh);    // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Rope Maze Wall Size and Visibility Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetWallHeight(const std::shared_ptr<rclcpp::Node>& nh);           // METERS
    float GetWallCenterOfMassZ(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetOuterWallsAlpha(const std::shared_ptr<rclcpp::Node>& nh);      // 0.0 thru 1.0 (inclusive)
    float GetFloorDividerAlpha(const std::shared_ptr<rclcpp::Node>& nh);    // 0.0 thru 1.0 (inclusive)
    float GetFirstFloorAlpha(const std::shared_ptr<rclcpp::Node>& nh);      // 0.0 thru 1.0 (inclusive)
    float GetSecondFloorAlpha(const std::shared_ptr<rclcpp::Node>& nh);     // 0.0 thru 1.0 (inclusive)

    ////////////////////////////////////////////////////////////////////////////
    // Rope Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetRopeSegmentLength(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetRopeRadius(const std::shared_ptr<rclcpp::Node>& nh);           // METERS
    int GetRopeNumLinks(const std::shared_ptr<rclcpp::Node>& nh);
    float GetRopeExtensionVectorX(const std::shared_ptr<rclcpp::Node>& nh);
    float GetRopeExtensionVectorY(const std::shared_ptr<rclcpp::Node>& nh);
    float GetRopeExtensionVectorZ(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Rope starting position settings
    ////////////////////////////////////////////////////////////////////////////

    float GetRopeCenterOfMassX(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetRopeCenterOfMassY(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    float GetRopeCenterOfMassZ(const std::shared_ptr<rclcpp::Node>& nh);    // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Cloth settings
    ////////////////////////////////////////////////////////////////////////////

    float GetClothXSize(const std::shared_ptr<rclcpp::Node>& nh);           // METERS
    float GetClothYSize(const std::shared_ptr<rclcpp::Node>& nh);           // METERS
    float GetClothCenterOfMassX(const std::shared_ptr<rclcpp::Node>& nh);   // METERS
    float GetClothCenterOfMassY(const std::shared_ptr<rclcpp::Node>& nh);   // METERS
    float GetClothCenterOfMassZ(const std::shared_ptr<rclcpp::Node>& nh);   // METERS
    float GetClothLinearStiffness(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Cloth BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    int GetClothNumControlPointsX(const std::shared_ptr<rclcpp::Node>& nh);
    int GetClothNumControlPointsY(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Generic target patch settings
    ////////////////////////////////////////////////////////////////////////////

    float GetCoverRegionXMin(const std::shared_ptr<rclcpp::Node>& nh);  // METERS
    size_t GetCoverRegionXSteps(const std::shared_ptr<rclcpp::Node>& nh);
    float GetCoverRegionXRes(const std::shared_ptr<rclcpp::Node>& nh);  // METERS
    float GetCoverRegionYMin(const std::shared_ptr<rclcpp::Node>& nh);  // METERS
    size_t GetCoverRegionYSteps(const std::shared_ptr<rclcpp::Node>& nh);
    float GetCoverRegionYRes(const std::shared_ptr<rclcpp::Node>& nh);  // METERS
    float GetCoverRegionZMin(const std::shared_ptr<rclcpp::Node>& nh);  // METERS
    size_t GetCoverRegionZSteps(const std::shared_ptr<rclcpp::Node>& nh);
    float GetCoverRegionZRes(const std::shared_ptr<rclcpp::Node>& nh);  // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    size_t GetNumSimstepsPerGripperCommand(const std::shared_ptr<rclcpp::Node>& nh);
    float GetSettlingTime(const std::shared_ptr<rclcpp::Node>& nh, const float default_time = 4.0);
    double GetTFWaitTime(const std::shared_ptr<rclcpp::Node>& nh, const double default_time = 4.0);

    ////////////////////////////////////////////////////////////////////////////
    // Robot settings
    ////////////////////////////////////////////////////////////////////////////

    double GetRobotControlPeriod(const std::shared_ptr<rclcpp::Node>& nh);      // SECONDS
    double GetMaxGripperVelocityNorm(const std::shared_ptr<rclcpp::Node>& nh);  // SE(3) velocity
    double GetMaxDOFVelocityNorm(const std::shared_ptr<rclcpp::Node>& nh);      // rad/s

    ////////////////////////////////////////////////////////////////////////////
    // World size settings for Graph/Dijkstras - DEFINED IN BULLET FRAME, but WORLD SIZES
    ////////////////////////////////////////////////////////////////////////////

    double GetWorldXStep(const std::shared_ptr<rclcpp::Node>& nh);              // METERS
    double GetWorldXMinBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    double GetWorldXMaxBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    int64_t GetWorldXNumSteps(const std::shared_ptr<rclcpp::Node>& nh);
    double GetWorldYStep(const std::shared_ptr<rclcpp::Node>& nh);              // METERS
    double GetWorldYMinBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    double GetWorldYMaxBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    int64_t GetWorldYNumSteps(const std::shared_ptr<rclcpp::Node>& nh);
    double GetWorldZStep(const std::shared_ptr<rclcpp::Node>& nh);              // METERS
    double GetWorldZMinBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    double GetWorldZMaxBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);    // METERS
    int64_t GetWorldZNumSteps(const std::shared_ptr<rclcpp::Node>& nh);
    double GetWorldResolution(const std::shared_ptr<rclcpp::Node>& nh);         // METERS

    // Is used as a scale factor relative to GetWorldResolution.
    // The resulting voxel sizes in the SDF are
    // GetWorldResolution() / GetSDFResolutionScale() in size.
    int GetSDFResolutionScale(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner trial type settings
    ////////////////////////////////////////////////////////////////////////////

    TrialType GetTrialType(const std::shared_ptr<rclcpp::Node>& nh);
    MABAlgorithm GetMABAlgorithm(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Diminishing Rigidity Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetDefaultDeformability(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Adaptive Jacobian Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetAdaptiveModelLearningRate(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Constraint Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetConstraintTranslationalDir(const std::shared_ptr<rclcpp::Node>& nh);
    double GetConstraintTranslationalDis(const std::shared_ptr<rclcpp::Node>& nh);
    double GetConstraintRotational(const std::shared_ptr<rclcpp::Node>& nh);
    double GetConstraintTranslationalOldVersion(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Bandit Multi-model settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetCollectResultsForAllModels(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRewardScaleAnnealingFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRewardScaleFactorStart(const std::shared_ptr<rclcpp::Node>& nh);
    double GetProcessNoiseFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetObservationNoiseFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetCorrelationStrengthFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetDeformabilityRangeMin(const std::shared_ptr<rclcpp::Node>& nh);
    double GetDeformabilityRangeMax(const std::shared_ptr<rclcpp::Node>& nh);
    double GetDeformabilityRangeStep(const std::shared_ptr<rclcpp::Node>& nh);
    double GetAdaptiveLearningRateRangeMin(const std::shared_ptr<rclcpp::Node>& nh);
    double GetAdaptiveLearningRateRangeMax(const std::shared_ptr<rclcpp::Node>& nh);
    double GetAdaptiveLearningRateRangeStep(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetUseRandomSeed(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetPlannerSeed(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner - Stuck detection settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetEnableStuckDetection(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetNumLookaheadSteps(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRubberBandOverstretchPredictionAnnealingFactor(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetMaxGrippersPoseHistoryLength(const std::shared_ptr<rclcpp::Node>& nh);
    double GetErrorDeltaThresholdForProgress(const std::shared_ptr<rclcpp::Node>& nh);
    double GetGrippersDistanceDeltaThresholdForProgress(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner - RRT settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetRRTReuseOldResults(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetRRTStoreNewResults(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTHomotopyDistancePenalty();
    double GetRRTBandDistance2ScalingFactor(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetRRTBandMaxPoints(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTMaxRobotDOFStepSize(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTMinRobotDOFStepSize(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTMaxGripperRotation(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTGoalBias(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTBestNearRadius(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTFeasibilityDistanceScaleFactor(const std::shared_ptr<rclcpp::Node>& nh);
    int64_t GetRRTMaxShortcutIndexDistance(const std::shared_ptr<rclcpp::Node>& nh);
    uint32_t GetRRTMaxSmoothingIterations(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTSmoothingBandDistThreshold(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTTimeout(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetRRTNumTrials(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTPlanningXMinBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTPlanningXMaxBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTPlanningYMinBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTPlanningYMaxBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTPlanningZMinBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);
    double GetRRTPlanningZMaxBulletFrame(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetUseCBiRRTStyleProjection(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetRRTForwardTreeExtendIterations(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetRRTBackwardTreeExtendIterations(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetRRTUseBruteForceNN(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetRRTKdTreeGrowThreshold(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetRRTTestPathsInBullet(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Transition Learning Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetTransitionMistakeThreshold(const std::shared_ptr<rclcpp::Node>& nh);
    double GetTransitionDefaultPropagationConfidence(const std::shared_ptr<rclcpp::Node>& nh);
    double GetTransitionDefaultBandDistThreshold(const std::shared_ptr<rclcpp::Node>& nh);
    double GetTransitionConfidenceThreshold(const std::shared_ptr<rclcpp::Node>& nh);
    double GetTransitionTemplateMisalignmentScaleFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetTransitionTightenDeltaScaleFactor(const std::shared_ptr<rclcpp::Node>& nh);
    double GetTransitionHomotopyChangesScaleFactor(const std::shared_ptr<rclcpp::Node>& nh);

    ClassifierType GetClassifierType(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Pure Jacobian based motion controller paramters
    ////////////////////////////////////////////////////////////////////////////

    bool GetJacobianControllerOptimizationEnabled(const std::shared_ptr<rclcpp::Node>& nh);
    double GetCollisionScalingFactor(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Stretching constraint controller parameters
    ////////////////////////////////////////////////////////////////////////////

    StretchingConstraintControllerSolverType GetStretchingConstraintControllerSolverType(const std::shared_ptr<rclcpp::Node>& nh);
    int64_t GetMaxSamplingCounts(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetUseFixedGripperDeltaSize(const std::shared_ptr<rclcpp::Node>& nh);
    double GetStretchingCosineThreshold(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetVisualizeOverstretchCones(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Straight line motion parameters for testing model accuracy
    // Note: these parameters are gripper velocities *in gripper frame*
    ////////////////////////////////////////////////////////////////////////////

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double, 6, 1>>>
    GetGripperDeltaTrajectory(const std::shared_ptr<rclcpp::Node>& nh, const std::string& gripper_name);
    double GetGripperStraightLineMotionTransX(const std::shared_ptr<rclcpp::Node>& nh);
    double GetGripperStraightLineMotionTransY(const std::shared_ptr<rclcpp::Node>& nh);
    double GetGripperStraightLineMotionTransZ(const std::shared_ptr<rclcpp::Node>& nh);
    double GetGripperStraightLineMotionAngularX(const std::shared_ptr<rclcpp::Node>& nh);
    double GetGripperStraightLineMotionAngularY(const std::shared_ptr<rclcpp::Node>& nh);
    double GetGripperStraightLineMotionAngularZ(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    bool GetBanditsLoggingEnabled(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetControllerLoggingEnabled(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetLogFolder(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetDataFolder(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetDijkstrasStorageLocation(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetCollisionMapStorageLocation(const std::shared_ptr<rclcpp::Node>& nh);
    bool GetScreenshotsEnabled(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetScreenshotFolder(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetTestRobotMotionTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetExecuteRobotMotionTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetTestRobotMotionMicrostepsTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGenerateTransitionDataTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetTestRobotPathsTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetWorldStateTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetCoverPointsTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetCoverPointNormalsTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetMirrorLineTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetFreeSpaceGraphTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetSignedDistanceFieldTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripperNamesTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripperAttachedNodeIndicesTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripperStretchingVectorInfoTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripperPoseTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetRobotConfigurationTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetObjectInitialConfigurationTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetObjectCurrentConfigurationTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetRopeCurrentNodeTransformsTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetVisualizationMarkerTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetVisualizationMarkerArrayTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetClearVisualizationsTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetConfidenceTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetConfidenceImageTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripperCollisionCheckTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetRestartSimulationTopic(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetTerminateSimulationTopic(const std::shared_ptr<rclcpp::Node>& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Live Robot Settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetGripper0Name(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripper1Name(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripper0TFName(const std::shared_ptr<rclcpp::Node>& nh);
    std::string GetGripper1TFName(const std::shared_ptr<rclcpp::Node>& nh);
    size_t GetGripperAttachedIdx(const std::shared_ptr<rclcpp::Node>& nh, const std::string& gripper_name);

    ////////////////////////////////////////////////////////////////////////////
    // ROS TF Frame name settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetBulletFrameName();
    std::string GetTaskFrameName();
    std::string GetWorldFrameName();
    std::string GetTableFrameName();
}

#endif // ROS_PARAMS_HPP
