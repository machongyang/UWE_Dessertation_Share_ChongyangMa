#include <iostream>
#include <vtkPLYReader.h>
#include <vtkTriangleFilter.h>
#include <vtkSmartPointer.h>
#include <vtkMassProperties.h>

using namespace std;


int main(int argc, char** argv)
{
	string inputFilename = "3_H_B.ply";


	//vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();

	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(inputFilename.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
	tri->SetInputData(reader->GetOutput());
	tri->Update();
	vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
	poly->SetInputData(tri->GetOutput());
	poly->Update();

	double vol = poly->GetVolume();
	double area = poly->GetSurfaceArea();

	cout << "Volume" << vol << endl;
	cout << "Surface area" << area << endl;

	system("pause");

	return (0);
}
