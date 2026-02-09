import sys
import os
import logging

def run_ik(project_dir, use_simple_model=True):
    """
    Wrapper to run Pose2Sim kinematics.
    
    Args:
        project_dir (str): output_dir where 'pose-3d' folder contains TRC files.
        use_simple_model (bool): Use simple model (faster, no muscles).
    """
    
    # Mock importlib.metadata.version if needed
    try:
        from importlib.metadata import version
        try:
            version("pose2sim")
        except:
            # Inject mock
            import importlib.metadata
            def mock_version(name):
                return "0.0.0"
            importlib.metadata.version = mock_version
    except ImportError:
        pass

    # Add Pose2Sim to path if not installed
    # Assuming standard location or reliant on PYTHONPATH
    # If the user provided paths in launch file, we might need to add them here 
    # but for now assume they are in PYTHONPATH or installed in the env.
    
    try:
        from Pose2Sim import Pose2Sim
        from Pose2Sim.kinematics import kinematics_all
    except ImportError:
        logging.error("Pose2Sim not found. Make sure it is in PYTHONPATH.")
        return

    # Construct Config Dict
    config_dict = {
        "project": {
            "project_dir": project_dir,
            "participant_height": 1.75, # Default, or can be 'auto'
            "participant_mass": 70.0
        },
        "pose": {
            "pose_model": "BODY_WITH_FEET" # Matches 25 keypoints? 
            # rtmlib 'balanced' body_26? Need to align this with what rtmlib outputs and what TRC header has.
            # If we use 17 keypoints (COCO), use 'BODY'
        },
        "kinematics": {
            "use_augmentation": False,
            "use_simple_model": use_simple_model,
            "right_left_symmetry": True,
            "remove_individual_scaling_setup": True,
            "remove_individual_ik_setup": True,
            "fastest_frames_to_remove_percent": 0.1,
            "large_hip_knee_angles": 45,
            "trimmed_extrema_percent": 0.5,
            "close_to_zero_speed_m": 0.2,
            "default_height": 1.75
        }
    }
    
    # Run Kinematics
    logging.info(f"Running Pose2Sim Kinematics in {project_dir}")
    try:
        kinematics_all(config_dict)
        logging.info("Kinematics finished.")
    except Exception as e:
        logging.error(f"Pose2Sim Kinematics failed: {e}")
