package frc.robot.util;

import java.net.NetworkInterface;
import java.util.Enumeration;

public class MacAddress {
    public static final String SHADOW_BEETLE_MAC = "00-80-2F-37-42-0D";
    public static final String SECONDARY_ROBOT_ONE_MAC = "10-50-FD-C6-35-0D";
    public static final String SWERVEBOT_2_MAC = "00-80-2F-36-F9-75";

    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder macAddress = new StringBuilder();
            while (networkInterface.hasMoreElements()) {
                NetworkInterface tempInterface =
                        networkInterface.nextElement(); // Instantiates the next element in our network interface
                if (tempInterface != null) {
                    byte[] mac = tempInterface.getHardwareAddress(); // Reads the MAC address from our NetworkInterface
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            // Formats our mac address by splitting it into two-character segments and hyphenating them
                            // (unless it is the final segment)
                            macAddress.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return macAddress.toString();
                    } else {
                        System.out.println("Address not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address not found");
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return "";
    }
}
