import sys
import os
import logging

def run_sports2d(video_path, output_dir):
    """
    Wrapper to run Sports2D on a video file.
    """
    # Mock importlib
    try:
        from importlib.metadata import version
        try:
            version("sports2d")
        except:
             import importlib.metadata
             def mock_version(name):
                 return "0.0.0"
             importlib.metadata.version = mock_version
    except ImportError:
        pass

    try:
        from Sports2D import Sports2D
    except ImportError:
        logging.error("Sports2D not found.")
        return

    # Construct Config Dict for Sports2D
    # Based on Sports2D documentation/examples
    config_dict = {
        "project": {
            "video_input": video_path,
            "project_dir": output_dir, # Sports2D creates subfolders here
            "video_dir": os.path.dirname(video_path),
            "participant_height": 1.75, 
            "participant_mass": 70.0
        },
        "process": {
            "multiperson": False,
            "mode": "lightweight", # User preference or default?
            "show_realtime_results": False,
            "save_vid": True,
            "save_img": False,
            "save_pose": True,
            "save_trc": True, # Crucial
            "save_mot": True, # Crucial
            "save_v3d": False
        },
        "pose": {
            "pose_model": "BODY_25B" # or BODY_25?
        }
        # Add other sections as needed (detect, tracking, etc.)
    }

    logging.info(f"Running Sports2D on {video_path} with output to {output_dir}")
    
    try:
        from Sports2D import Sports2D
        Sports2D.process(config_dict)
        logging.info("Sports2D processing finished.")
    except Exception as e:
        logging.error(f"Sports2D process execution failed: {e}")

