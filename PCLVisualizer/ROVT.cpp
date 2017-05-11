#include "ROVT.h"
#include "Glb.h"

//�½���������
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	std::cout << "i only run once" << std::endl;
}
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{  //ʹ�ÿո�������ӵ�����������������ʾ
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}