import Sofa.Gui
import importlib

#import sofagym
#from sofagym.no_server import *

scene = "CartPole"
#scene = "CatheterBeam"
#scene = "CatheterBeam1Instrument"

createScene = importlib.import_module("sofagym.envs."+scene+"." + scene + "Scene").createScene


def main():
    #Create the root node
    root = Sofa.Core.Node("root")

    # Call the below 'createScene' function to create the scene graph
    createScene(root)
    Sofa.Simulation.init(root)

    # Launch the GUI (qt or qglviewer)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1280, 720)
    
    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

    print("Simulation is done.")


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
