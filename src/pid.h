#ifndef PID_H
#define PID_H

constexpr double kInfinity = 1.7e308;
constexpr double kInfinityNeg = -kInfinity;

/*
 * PD Controller
 */
class PD {
 public:
  PD(double k_p, double k_d, 
     double min_out = kInfinityNeg, double max_out = kInfinity);
  virtual ~PD() = default;

  double Update(double error);
  void Reset();
  virtual double Get() const;

 protected:
  double p_error_ = 0.0;
  double d_error_ = 0.0;
  double coeff_p_ = 0.0;
  double coeff_d_ = 0.0;
  double min_out_ = kInfinityNeg;
  double max_out_ = kInfinity;
};

/*
 * PD Controller where P has an exponent
 */
class PDPow : public PD {
 public:
  PDPow(double k_p, double k_d, double k_e = 1.0,
        double min_out = kInfinityNeg, double max_out = kInfinity);
  virtual ~PDPow() = default;

  double Get() const override;

 protected:
  double exp_ = 1.0;
};

/*
 * PID Controller
 */
class PID {
 public:
  PID(double k_p, double k_i, double k_d, 
      double min_out = kInfinityNeg, double max_out = kInfinity);
  virtual ~PID() = default;

  double Update(double error, double time = 1.0);
  void Reset();
  double Get() const;

 protected:
  double p_error_ = 0.0;
  double i_error_ = 0.0;
  double d_error_ = 0.0;
  double coeff_p_ = 0.0;
  double coeff_i_ = 0.0;
  double coeff_d_ = 0.0;
  double min_out_ = kInfinityNeg;
  double max_out_ = kInfinity;
};

#endif  // PID_H
