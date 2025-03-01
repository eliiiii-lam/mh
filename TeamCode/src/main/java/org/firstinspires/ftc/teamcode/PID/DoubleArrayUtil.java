package org.firstinspires.ftc.teamcode.PID;

//This can probably be depreciated as I believe a circular array can do the same thing
public class DoubleArrayUtil {
    private int size;
    private double[] window;
    private int index = 0;
    private int count = 0;
    private double sum = 0;

    public DoubleArrayUtil(int size) {
        this.size = size;
        window = new double[size];
    }

    public void reset() {
        window = new double[size]; // Clear the window
        index = 0;                 // Reset index
        count = 0;                 // Reset count
        sum = 0;                   // Reset sum
    }

    public boolean allValuesSame() {
        if (count == 0) return false; // If no values have been added, return false

        double firstValue = window[0];
        for (int i = 1; i < count; i++) {
            if (window[i] != firstValue || !isWindowFull()) {
                return false;
            }
        }
        return true;
    }

    public boolean isWindowFull() {
        return count == size;
    }

    public void populateWindow(double value) {
        for (int i = 0; i < size; i++) {
            window[i] = value;
        }
        count = size;  // Mark the window as full
        sum = value * size;  // Update the sum accordingly
        index = 0;  // Reset index to 0
    }

    public double filter(double Num) {
        if (count < size) {
            sum += Num;
            window[index++] = Num;
            count++;
        } else {
            sum -= window[index];
            window[index++] = Num;
            sum += Num;
        }
        if (index == size) {
            index = 0;
        }
        return sum / count;
    }
}