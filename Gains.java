package frc.robot;

public class Gains {
    public final double P;
    public final double I;
    public final double D;
    public final double F;
    public final int IZONE;
    public final double PEAK_OUTPUT;
	
	public Gains(double _P, double _I, double _D, double _F, int _Izone, double _PeakOutput){
		P = _P;
		I = _I;
		D = _D;
		F = _F;
		IZONE = _Izone;
		PEAK_OUTPUT = _PeakOutput;
	}
}
