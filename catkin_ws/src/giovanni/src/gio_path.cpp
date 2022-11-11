#include <gio_path.h>

using namespace std;

void CGioController::InitDefault(){ //Initialiert den Controller mit Roboterspezifischen Werten
  this->loop_exit    = 0; 
  this->AXIS_LENGTH  = 0.485;
  this->Vm           = 1; // kleinste Geschwindigkeitsdifferenz, die von den Motoren verarbeitet werden können.
  this->d_y          = 0.001; // untere Schranke für y 
  this->d_th         = 0.34; //untere Schranke für theta
  this->kr_max       = 2/AXIS_LENGTH; //obere Schranke für die Krrümmung
  this->u0           = Vm/2.0; //Geschwindigkeit des Roboters
  this->a            = 1.2; //Konstante, die Aussagen über das einschwingverhalten trifft
  this->epsilon      = 1e-4; 
  this->path         = new CCurve(); //CCurve Object, das die Punkte des Pfades und hilfreiche Methoden beinhaltet
  this->setPose(0.0, 0.0, 0); //initialisiert die Position
  this->setLocalSystem(0.0); //initialisiert die Felder der Rotationsmatrix, die verwendet wird um zwischen globalem Koordinatensystem und Koordinatensystem des Roboters zu wechseln
}

void  CGioController::setLocalSystem(double ang){ //Berechnet die Felder der Rotationsmatrix, die verwendet wird um zwischen globalem Koordinatensystem und Koordinatensystem des Roboters zu wechseln
  ex[0] = cos(ang); 
  ex[1] = sin(ang); 
  ey[0] = -sin(ang); 
  ey[1] = cos(ang); 
}

double  CGioController::H_case_1(double y, double theta, double u, double alpha, double *gamma){ //Formel für den Fall, dass sich der Roboter auf (bzw. sehr nah an) der Referenzgeraden befindet
  double h_j = SQR(kr_max) / (SQR(theta) * SQR(1+2*alpha));
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

double  CGioController::H_case_2(double y, double theta, double u, double alpha, double *gamma) { //Formel für den Fall, dass sich der Roboter nicht auf (bzw. sehr nah an) der Referenzgeraden befindet
  double h_j = 0.5 * ( - SQR(theta)/SQR(y) + sqrt( SQR(SQR(theta)/SQR(y)) + 4 * SQR(kr_max) / ( SQR(y) * SQR(1 + 2*alpha) ) ) );
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

double  CGioController::Compute_W(double y, double theta, double a, double u, int *err){ //berechnet die Winkelgeschwindigkeit nach Giovanni Invideri und Maria Corridini
  double res, h, gamma;

  if(fabs(y) < epsilon) { //der If-Block sorgt dafür dass kein |y| < d_y oder |theta| < d_th zur Berechnung von h verwendet wird
    if(fabs(theta) >= d_th) {
     cerr << "H1_1\n";
	 h = H_case_1(y, theta, u, a, &gamma);
	 *err = 0;
    } else {
    cerr << "H1_2\n";
	 h = H_case_1(y, d_th, u, a, &gamma);
	 *err = 0;
    }
  } else {
    if(fabs(y) >= d_y) {
	 h = H_case_2(y, d_th, u, a, &gamma);
	 *err = 0;
    } else {
	 if(fabs(theta) < d_th) {
	   h = H_case_2(d_y, d_th, u, a, &gamma);
	   *err = 0;
	 } else {
	   h = H_case_2(d_y, theta, u, a, &gamma);
	   *err = 0;
	 }
    }
  }

  if(fabs(theta) >= d_th) { //ist theta klein genug, kann die Kleinwinkelnäherung sin(theta) = theta angewandt werden.
    res = -h * u * y * sin(theta)/theta - gamma * theta;
  } else {
    if(fabs(theta) <= d_th && fabs(theta) >= d_th_0){
	 res = -h * u * y - gamma * theta;
    } else {
	 res = -h * u * y;
    }
  }

  return res;
}

CGioController::CGioController(){ //erstellt ein CGioControllerobjekt
  this->InitDefault();
  giofile.open("pos.dat");
};

CGioController::~CGioController(){ //löscht das CGioControllerobjekt
  giofile.flush();
  giofile.close();
  giofile.clear();  
  delete path;
}
    
void CGioController::setAxisLength(double val) { //setzt AXIS_LENGTH
  if(fabs(val) > 0) {
    this->AXIS_LENGTH = fabs(val);
    this->kr_max = 2/AXIS_LENGTH;
  }
}

double CGioController::getAxisLength() { //gibt AXIS_LENGTH zurück
  return this->AXIS_LENGTH;
}

void CGioController::setCurrentVelocity(double val, int abs) { //setzt u0 und Vm
  if(abs) {
    this->u0 = val;
  } else {
    this->u0 = (this->u0>val)?(val+this->u0):(this->u0+val);
  }
  this->Vm = this->u0 * 2.0;
}

double CGioController::getCurrentVelocity() { //gibt u0 zurück
  return this->u0;
}

void CGioController::setPose(double x, double y, double phi) { //setzt x,y und phi
  this->x0 = x;
  this->y0 = y;
  this->phi0 = phi;
  NormalizeAngle(this->phi0);
}

void CGioController::getPose(double &x, double &y, double &ph) { //setzt die Parameter auf die Werte, die im Controller gespeichert sind
  x = this->x0;
  y = this->y0;
  ph = this->phi0;
}

int CGioController::getPathFromFile(const char* fname){ //liest die fname.dat als CCurve ein
  int res = path->LoadFromFile(fname);
  if(res){
    path->initTraversal();
    this->loop_exit = path->getNext();
  }
  return res;
}

/*
 * Ab ist es hilfreich zu verstehen, die der Controller grundsätzlich funktioniert:
 * Der Pfad wird in Form einer Liste an x und y Koordinaten gespeichert (2 Arrays).
 * Es werden immer zwei Punkte (Ankerpunkte) ausgewählt, mit denen eine Gerade (Referenzgerade) konstruiert wird.
 * Der in der Lister vordere Punkt wird erster Ankerpunkt genanne, der hintere zweiter Ankerpunkt.
 * Die Punke sind immer aufeinanderfolgend in der Liste, bzw in den beiden Arrays. (Grenzfall für looped != 0 kann einer ganz am Anfang und der andere ganz am Ende sein)
 * Die Gerade dient also Referenzpfad, bis die nächsten beiden Punkte zur Parametrisierung der Geraden verwendet werden.
 * Wenn die Gerade gewechselt wird, wird der zweite Ankerpunkt zum ersten Ankerpunkt und der zweite Ankerpunkt wird.
 */

int CGioController::canDetermineRobotPosition(int looped){ //gibt 1 zurück, wenn der Roboter am Ziel angekommen ist, sonst 0
  int exit;
  int check_prev;

  exit       = 0;
  check_prev = 0;
/*
 *looped solange, bis
 * a) der Pfad am letzten Punkt angekommen ist
 * b) eine Sekante gefunden wurde, die als Referenzgerade in Frage kommt
 */
  while(this->loop_exit != 0 && !exit) { //exit == 1 falls der Roboter weitgenug vom zweiten Ankerpunkt der Referenzgeraden entfernt ist, loop_exit == 1 falls jeder Punkt des Pfads 2*(looped-1) oft als Ankerpunkt verwendet wurde (jeder durchlauf 2 mal)
    if(path->pointInn(x0,y0)) { //pointInn(x0,y0) gibt 1 zurück, wenn die momentane Position nicht vor dem zweiten Ankerpunkt ist. Die Position wird dafür auf die Referenzgerade projeziert und geprüft, ob die Projektion weiter vom ersten Ankerpunkt entfernt ist als der zweite Ankerpunkt
	 if(path->getDistanceToEnd(x0, y0) > 0.1) {  //gibt die Distanz von der Projektion von (x0,y0) auf die Referenzgerade zum zweiten Ankerpunkt an
	   exit = 1;
	   giofile << path->getDistanceToEnd(x0, y0) << " ";
	   continue;
	 }
    }
    this->loop_exit = path->getNext(looped); //gibt 1 zurück, falls der Roboter am Ziel angekommen ist, sonst wird der ptr auf den ersten Ankerpunkt inkrementiert und 0 zurückgegeben
  }
	
  return exit;
}

int CGioController::getNextState(double &u, double &w, double &vleft, double &vright, int looped){ //Hauptmethode eines CGioController Objekts. Berechnet die Winkelgeschwindigkeit 
  double l, phic = 0, pathAng = 0, tmpw;
  int err;
  
  if(!canDetermineRobotPosition(looped)) { //für canDetermineRObotPosition == 1 ist der Roboter am Ziel, also wird die Fahrt gestoppt
    u = 0;
    w = 0;
    vleft = 0;
    vright = 0;
    return 0;
  }
	
  l = path->getDistance(x0,y0); //l ist die (momentan noch positive) Distanz zur orthogonalen Projektion der Position des Roboters auf die Referenzgerade
  if( path->Evaluate(x0,y0) > 5e-7 ) { //l muss auch negative Werte annehmen können
    l = -l;
  }
  
  giofile << l << " " << path->Evaluate(x0, y0) << " ";  // u 2 3

  phic = phi0 - path->getAng(); //phic ist der Winkel des Roboters zur Referenzgeraden. getAngle ist der Winkel der Referenzgeraden zur x-Achse des Globalen Koordinateensystems, während phi0 der Winkel des Roboters zur x-Achse des Globalen Koordinateensystems ist
  giofile << phi0 << " " << pathAng << " " << phic << " "; // using 4 5 6
  NormalizeAngle(phic); //stellt -pi <= phic <= pi sicher

  u = this->u0;

  w = Compute_W(l, phic, this->a, u, &err); //Berechnet die Winkelgeschwindigkeit
  double sign = w < 0 ? -1 : 1;
  w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? sign*(this->Vm / this->AXIS_LENGTH) : w; //stellt sicher, dass die untere Schranke für die Geschwindigkeitsdifferenz der beiden Räder, Vm, eingehalten wird
  //w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? fabs(this->Vm / this->AXIS_LENGTH) : w;

  giofile << w << " ";
  vright = u - AXIS_LENGTH * w * 0.5; //berechnet die Geschwindigkeit des linken Rads
  vleft  = u + AXIS_LENGTH * w * 0.5; //berechnet die Geschwindigkeit des rechten Rads
  return 1; //gibt 1 zurück um zu signalisieren, dass nicht schief gelaufen ist und der Roboter noch nicht sein Ziel erreicht hat
}
