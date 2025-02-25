package org.firstinspires.ftc.teamcode.SubSytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class TestAction {

    public TestAction() {}

    /* ************************* Actions * *************************/
    public class Counter implements Action {

        int target;
        int count;
        long previousTime;
        long currentTime;

        public Counter(int countToSeconds) {
            target = countToSeconds;
            previousTime = System.currentTimeMillis();
        }

        public int getCount() {
            return count;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            currentTime = System.currentTimeMillis();
            if (currentTime - previousTime >= 1000) {
                count +=  (int) (currentTime - previousTime) / 1000;
                previousTime = currentTime - (currentTime - previousTime) % 1000;
            }

            return !(count > target);
        }

    }

    public Action countTo(int seconds) {
        return new Counter(seconds);
    }

}
