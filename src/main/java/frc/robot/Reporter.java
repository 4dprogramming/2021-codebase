// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Reporter {
    public enum DataType{
        STRING,
        INT,
        DOUBLE,
        BOOLEAN,
        SENDABLE
    }
    public class ReportData{
        DataType dataType;
        String sData;
        double dData;
        int iData;
        boolean bData;
        Sendable sendableData;
        public ReportData(double data){
            dataType = DataType.DOUBLE;
            this.dData = data;
        }
        public ReportData(Sendable data){
            dataType = DataType.SENDABLE;
            this.sendableData = data;
        }
        public ReportData(String data){
            dataType = DataType.STRING;
            this.sData = data;
        }
        public ReportData(int data){
            dataType = DataType.INT;
            this.iData = data;
        }
        public ReportData(boolean data){
            dataType = DataType.BOOLEAN;
            this.bData = data;
        }
    }
    public class Report{
        String name;
        Supplier<ReportData> supplier;
        public Report(String _name, Supplier<ReportData> _supplier){
            this.name = _name;
            this.supplier = _supplier;
        }
    }

    private ArrayList<Report> reports = new ArrayList<>();

    public void addReport(Report report){
        reports.add(report);
    }

    public void reportToDashboard(){
        for (int i = 0; i < reports.size(); i++){
            Report reportEntry = reports.get(i);
            String reportName = reportEntry.name;
            ReportData reportData = reportEntry.supplier.get();
            switch (reportData.dataType){
                case DOUBLE:
                    break;
                case BOOLEAN:
                    break;
                case INT:
                    break;
                case STRING:
                    break;
                case SENDABLE:
                    break;
            }
        }
    }
}
