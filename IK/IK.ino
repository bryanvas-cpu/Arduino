double joint_angles[4];
double l1 = 112.5, l2=225.0, l3=75, l4=150, l5=75, d=75;
double J2L = l1;
double J3L = l2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
}

int inverse_kinematics(double xe, double ye, double *output) {
  xe = -xe;
  double L = sqrtf(xe*xe + ye*ye);
  double J3 = acos(   ((l1 * l1) + (l2 * l2) - (L * L))   /   (2 * l1 * l2)   ) * (180 / PI);
  double B = acos(   ((L * L) + (l1 * l1) - (l2 * l2))   /   (2 * L * l1)   ) * (180 / PI);
  double A = atan(xe / ye) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
  double J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'
  joint_angles[0] = 90+J2;
  joint_angles[1] = J3 + J2 -90;
  double xa = l1*cos(joint_angles[0] * 3.1415/180.0) + l5*cos(joint_angles[1] * 3.1415/180.0 );
  double ya = l1*sin(joint_angles[0] * 3.1415/180.0) + l5*sin(joint_angles[1]* 3.1415/180.0 );
  Serial.print(" xa,ya "+ String(xa)+", " +String(ya) + " || ");


  xe = xa-d;
  ye = ya;

  L = sqrtf(xe*xe + ye*ye);
  J3 = acos(   ((l3 * l2) + (l4 * l4) - (L * L))   /   (2 * l3 * l4)   ) * (180 / PI);
  B = acos(   ((L * L) + (l3 * l3) - (l4 * l4))   /   (2 * L * l3)   ) * (180 / PI);
  A = atan(xe / ye) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
  J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

  joint_angles[2] = 90-J2;
  double k = asin(l3 * sin(J3) /L);
  joint_angles[3] = A + k +90;
  return 1;
}

void loop() {
  // put your main code here, to run repeatedly:
  // for(int i=150; i<300; i++){
  //   int result = inverse_kinematics(37.5, (double)i, joint_angles);
  //   Serial.println(String(joint_angles[0]) + " " + String(joint_angles[1]));
  //   delay(50);
  // }
  // for(int i=300; i>=150; i--){
  //   int result = inverse_kinematics(37.5, (double)i, joint_angles);
  //   Serial.println(String(joint_angles[0]) + " " + String(joint_angles[1]));
  //   delay(50);
  // }
  int result = inverse_kinematics(37.5, 200, joint_angles);
  Serial.println(String(joint_angles[0]) + " " + String(joint_angles[1]) + " " + String(joint_angles[2])+ " "+ String(joint_angles[3]));
}
