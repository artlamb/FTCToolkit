package utils;

    public enum Waypoint {
        START, WAYPOINT_1, WAYPOINT_2, PARK, UNKNOWN;

        public static String name(Waypoint state) {
            return values()[state.ordinal()].toString();
        }
    }
