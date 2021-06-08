#include <simpleRobot.h>

#define PI 3.14159

int main()
{

	cout.setf(std::ios::fixed, std::ios::floatfield);
	cout.precision(3);

	cout << "\n*** Fill Robot parameters:" << endl;
	cout << "\t >> const int dof = 4;" << endl;
	cout << "\t >> string joints = \"rppr\";  //r: revolute, p:prismatic" << endl;
	cout << "\t >> double data_theta[dof] = { 0, PI / 2, 0, 0 };" << endl;
	cout << "\t >> Vector tmp_theta(dof, data_theta);" << endl;
	cout << "\t >> double data_d[dof] = { 33, 0, 0, 26 };" << endl;
	cout << "\t >> Vector tmp_d(dof, data_d);" << endl;
	cout << "\t >> ..." << endl;
	cout << "\t >> Robot *scara = new Robot(dof, joints, &tmp_theta, &tmp_d, &tmp_a, &tmp_alpha); // Pointer Object" << endl;
	cout << "\t >> cout << scara << endl; // Prints robots parameters" << endl;

	// Dimensions are in [cm] y [rad]
	const int dof = 4;
	string joints = "rppr";  //r: revolute, p:prismatic

	double data_theta[dof] = { 0, PI / 2, 0, 0 };
	Vector tmp_theta(dof, data_theta);

	double data_d[dof] = { 33, 0, 0, 26 };
	Vector tmp_d(dof, data_d);

	double data_a[dof] = { 0, 0, 0, 0 };
	Vector tmp_a(dof, data_a);

	double data_alpha[dof] = { 0, PI / 2, 0, 0 };
	Vector tmp_alpha(dof, data_alpha);

	Robot *scara = new Robot(dof, joints, &tmp_theta, &tmp_d, &tmp_a, &tmp_alpha);
	cout << scara << endl;


	cout << "\n*** Copy robot:" << endl;
	cout << "\t Robot *scara_cp = new Robot;" << endl;
	cout << "\t scara_cp = scara;" << endl;
	cout << "\t cout << scara_cp << endl;" << endl;
	Robot *scara_cp = new Robot;
	scara_cp = scara;
	cout << scara_cp << endl;
	
	cout << "\t >> Robot scara2(dof, joints, &tmp_theta, &tmp_d, &tmp_a, &tmp_alpha); // Object" << endl;
	cout << "\t >> cout << scara2 << endl; // Prints robots parameters" << endl;
	Robot scara2(dof, joints, &tmp_theta, &tmp_d, &tmp_a, &tmp_alpha);
	cout << scara2 << endl;

	cout << "\n*** Get Robot parameters:" << endl;
	cout << "\t >> Vector *theta_out;" << endl;
	cout << "\t >> theta_out = scara->getTheta(); // All the vector Theta" << endl;
	Vector *theta_out;
	theta_out = scara->getTheta();
	cout << theta_out << endl;

	cout << "\t >> double theta_1 = scara->getTheta(1); // Specific value theta" << endl;
	double theta_1 = scara->getTheta(1);
	cout << theta_1 << endl;

	system("pause");

	cout << "\n*** Denavit-Hartemberg matrix:" << endl;
	cout << "\t >> Matrix m3;" << endl;
	cout << "\t >> scara->denavit(&m3, PI / 2, 4, 21, PI / 2);" << endl;
	Matrix m3;
	scara->denavit(&m3, PI / 2, 4, 21, PI / 2);
	cout << m3 << endl;

	system("pause");

	cout << "\n*** Rotation matrix in x-axes:" << endl;
	cout << "\t >> scara->rotx(&m,90); // In deg" << endl;
	Matrix m;
	scara->rotx(&m, 90); // In deg
	cout << m << endl;

	cout << "\n*** Rotation matrix in y-axes:" << endl;
	cout << "\t >> scara->roty(&m,90); // In deg" << endl;
	scara->roty(&m, 90); // In deg
	cout << m << endl;

	cout << "\n*** Rotation matrix in z-axes:" << endl;
	cout << "\t >> scara->rotz(&m,90); // In deg" << endl;
	scara->rotz(&m, 90); // In deg
	cout << m << endl;

	system("pause");

	cout << "\n*** Rotation matrix to Homogeneous Matrix:" << endl;
	cout << "\t >> scara->r2t(&m);" << endl;
	scara->r2t(&m);
	cout << m << endl;

	cout << "\n*** Homogeneous Matrix to Rotation matrix:" << endl;
	cout << "\t >> scara->t2r(&m);" << endl;
	scara->t2r(&m);
	cout << m << endl;

	system("pause");

	cout << "\n*** Forward Kinematics of the End Effector:" << endl;
	cout << "\t >> scara->fKine(&m, q);" << endl;

	double data_q[4] = { PI / 2, 2, 2, -PI / 3 };
	Vector q(4, data_q);
	scara->fKine(&m, &q);
	cout << m << endl;

	cout << "\n*** Forward Kinematics of the Joint N:" << endl;
	cout << "\t >> int n = 3;" << endl;
	cout << "\t >> scara->fKineN(&m, q, n);" << endl;

	int n = 3;
	Matrix m2;
	scara->fKineN(&m2, &q, n);
	cout << m2 << endl;

	system("pause");

	cout << "\n*** Transform difference to differential representation between 2 homogeneus matrix:" << endl;
	cout << "\t >> scara->tr2diff(&d, m, m2);" << endl;
	Vector d;
	scara->tr2diff(&d, &m, &m2);
	cout << d << endl;

	cout << "\n*** Transform difference to differential representation of the same homogeneus matrix:" << endl;
	cout << "\t >> scara->tr2diff(&d, m);" << endl;
	scara->tr2diff(&d, &m);
	cout << d << endl;

	system("pause");

	cout << "\n*** Jacobian Matrix:" << endl;
	cout << "\t >> " << endl;
	double data_q2[4] = { 0.8, 12, 3, 0.34 };
	Vector q2(4, data_q2);
	scara->Jacob0(&m2, &q2);
	cout << m2 << endl;

	system("pause");

	return 0;

}