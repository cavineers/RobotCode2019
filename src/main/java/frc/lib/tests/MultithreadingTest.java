package frc.lib.tests;

class MultithreadingTest {

    static int count = 0;

    public static void main(String[] args) {
        System.out.println("hi");
        Thread clockSync = new Thread() {
            public void run() {
                while (shouldContinue(count)) {
                    count++;
                    System.out.println(count);
                }
            }
        };
        clockSync.start();
        System.out.println(clockSync.isAlive());
        System.out.println(clockSync.isAlive());
    }

    public static boolean shouldContinue(int count) {
        return count < 100;
    }
}