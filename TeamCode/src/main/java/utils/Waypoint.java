package utils;

    public enum Waypoint {
        START, SHOOT_1, PICKUP_1, PICKUP_END_1, SHOOT_2, PICKUP_2, SHOOT_3, PICKUP_3, PARK, UNKNOWN;

        public static String name(Waypoint state) {
            return values()[state.ordinal()].toString();
        }
    }
