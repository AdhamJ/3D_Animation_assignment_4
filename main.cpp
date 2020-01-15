
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"



int main(int argc, char *argv[])
{
  //-----------------------------------
  bool enable_simplefication = false;		// enables and disables simplefication option
	bool render_arm_ass3 = false;					// starts IK animation
	bool collision_detection_ass4 = true;// starts collision detection simulation
	bool snake_game = false;								// starts snake game
	//-----------------------------------
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	if (snake_game) viewer.init_game();
	else viewer.load_mesh_from_configuration(enable_simplefication, render_arm_ass3, collision_detection_ass4);
	Init(*disp);
	renderer.init(&viewer);
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);
	delete disp;
}
