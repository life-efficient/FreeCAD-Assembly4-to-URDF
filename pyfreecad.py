import json
import logging
import xmlrpc.client
from contextlib import asynccontextmanager
from typing import AsyncIterator, Dict, Any, Literal

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("FreeCADMCPserver")


_only_text_feedback = False


class FreeCADConnection:
    def __init__(self, host: str = "localhost", port: int = 9875):
        host = "localhost"
        port = 9875
        self.server = xmlrpc.client.ServerProxy(f"http://{host}:{port}", allow_none=True)

    def ping(self) -> bool:
        return self.server.ping()

    def execute_code(self, code: str) -> dict[str, Any]:
        return self.server.execute_code(code)


def get_freecad_connection():
    """Get or create a persistent FreeCAD connection"""
    print(f"Connecting to FreeCAD")
    freecad_connection = FreeCADConnection(host="localhost", port=9875)
    if not freecad_connection.ping():
        logger.error("Failed to ping FreeCAD")
        freecad_connection = None
        raise Exception(
                "Failed to connect to FreeCAD. Make sure the FreeCAD addon is running."
            )
    else:
        print("Connected to FreeCAD")
        return freecad_connection
    global _freecad_connection
    if _freecad_connection is None:
        _freecad_connection = FreeCADConnection(host="localhost", port=9875)
        if not _freecad_connection.ping():
            logger.error("Failed to ping FreeCAD")
            _freecad_connection = None
            raise Exception(
                "Failed to connect to FreeCAD. Make sure the FreeCAD addon is running."
            )
    return _freecad_connection

def add_screenshot_if_available(response, screenshot):
    """Safely add screenshot to response only if it's available"""
    return response
    if screenshot is not None and not _only_text_feedback:
        response.append(ImageContent(type="image", data=screenshot, mimeType="image/png"))
    elif not _only_text_feedback:
        # Add an informative message that will be seen by the AI model and user
        response.append(TextContent(
            type="text", 
            text="Note: Visual preview is unavailable in the current view type (such as TechDraw or Spreadsheet). "
                 "Switch to a 3D view to see visual feedback."
        ))
    return response

# Helper function to safely add screenshot to response
def execute_code(code: str):
    """Execute arbitrary Python code in FreeCAD.

    Args:
        code: The Python code to execute.

    Returns:
        A message indicating the success or failure of the code execution, the output of the code execution, and a screenshot of the object.
    """
    freecad = get_freecad_connection()
    res = freecad.execute_code(code)
    # screenshot = freecad.get_active_screenshot()
    screenshot = None
    return res['message']
    try:
        
        if res["success"]:
            response = [
                TextContent(type="text", text=f"Code executed successfully: {res['message']}"),
            ]
            return add_screenshot_if_available(response, screenshot)
        else:
            response = [
                TextContent(type="text", text=f"Failed to execute code: {res['error']}"),
            ]
            return add_screenshot_if_available(response, screenshot)
    except Exception as e:
        logger.error(f"Failed to execute code: {str(e)}")
        return [
            TextContent(type="text", text=f"Failed to execute code: {str(e)}")
        ]

def main():
    execute_code("print('Hello, World!')")

if __name__ == "__main__":
    main()