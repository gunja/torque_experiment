#pragma once

class ECNWrapper {
    public:
        virtual void fillUINT32( const int slave, const int slot, const int subslot, uint32_t &dst) = 0;
        virtual bool MotorIndexOperationAllowed(int) const = 0;
        virtual void disableMotor(int) = 0;
        virtual void startDurableRotation(int mtr, double v, double d) = 0;
        virtual void clearError(int mtr) = 0;
        virtual void setAcceleration(int mtr, int value) = 0;
        virtual void setDeceleration(int mtr, int value) = 0;

        virtual void startGoToPosition(int mtr, int position, int velocity) = 0;
};

