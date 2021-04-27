

// int controller(float currentAngle);


class Controller{
    public:
        Controller();
        ~Controller();

        void initialize();
        int computeOutput(float currentAngle, float angularVel);

        float getKp();
        float getKi();
        float getKd();
        float getPAction();
        float getIAction();
        float getDAction();
        

        void setKp(float newKp);
        void setKi(float newKi);
        void setKd(float newKd);
        void setGains(float newKp, float newKi, float newKd);

    private:
        float _kp, _kd, _ki;
        float _pAction, _iAction, _dAction;

        float _angle, _previous_angle;
        float _setpoint, _error;
        
        float _integral, _angularVel;
        float _lastTime;
        
        int _output;
};