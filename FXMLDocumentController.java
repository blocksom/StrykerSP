/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package hip.positioning.system;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.shape.Box;
import javafx.scene.shape.Line;
import javafx.scene.transform.Rotate;
import javafx.stage.Modality;
import javafx.stage.Popup;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import jssc.SerialPort;
import static jssc.SerialPort.MASK_RXCHAR;
import jssc.SerialPortEvent;
import jssc.SerialPortException;
import jssc.SerialPortList;
import jssc.SerialPortTimeoutException;

/**
 *
 * @author Tristan
 */
public class FXMLDocumentController implements Initializable {

    // Arduino codes 
    SerialPort arduinoPort = null;
    ObservableList<String> portList;

    StringBuilder angle1 = new StringBuilder();
    StringBuilder angle2 = new StringBuilder();
    StringBuilder angle3 = new StringBuilder();
    StringBuilder angle4 = new StringBuilder();

    Rotate rxBox = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
    Rotate ryBox = new Rotate(0, 0, 0, 0, Rotate.Y_AXIS);
    Rotate rzBox = new Rotate(0, 0, 0, 0, Rotate.Z_AXIS);

    int count = 0;
    Boolean receivingMessage = false;
    String st;

    // views
    @FXML
    private Pane kneeFlexExtenView;

    @FXML
    private Pane intExtView;

    @FXML
    private Pane abAdView;

    @FXML
    private Pane hipFlexExtenView;

    // labels
    @FXML
    private Label intExtLabel;

    @FXML
    private Label kneeFlexExtenLabel;

    @FXML
    private Label hipFlexExtenLabel;

    @FXML
    private Label hipAbAdLabel;

    // buttons
    @FXML
    private RadioButton AdAbButton;

    @FXML
    private RadioButton intExtButton;

    @FXML
    private RadioButton hipflexExtButton;

    @FXML
    private RadioButton kneeFlexExtButton;

    @FXML
    private Box box;

    /*
        calibration method will present popups that will direct the user to 
        calibrate the sensors.
     */
    @FXML
    private void calibrate(ActionEvent event) {
        try {
            FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("Calibration.fxml"));
            Parent root1 = (Parent) fxmlLoader.load();
            Stage stage = new Stage();
            stage.initModality(Modality.APPLICATION_MODAL);
            stage.setTitle("Calibrating...");
            stage.setScene(new Scene(root1));
            stage.show();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /*
        These methods are for changing the viewing angles
     */
    @FXML
    private void changeToAddAb(ActionEvent event) {
        abAdView.setVisible(true);
        hipFlexExtenView.setVisible(false);
        intExtView.setVisible(false);
        kneeFlexExtenView.setVisible(false);
    }

    @FXML
    private void changeToHipFlexExten(ActionEvent event) {
        abAdView.setVisible(false);
        hipFlexExtenView.setVisible(true);
        intExtView.setVisible(false);
        kneeFlexExtenView.setVisible(false);
    }

    @FXML
    private void changeToIntExt(ActionEvent event) {
        abAdView.setVisible(false);
        hipFlexExtenView.setVisible(false);
        intExtView.setVisible(true);
        kneeFlexExtenView.setVisible(false);
    }

    @FXML
    private void changeToKneeFlexExten(ActionEvent event) {
        abAdView.setVisible(false);
        hipFlexExtenView.setVisible(false);
        intExtView.setVisible(false);
        kneeFlexExtenView.setVisible(true);
    }

    /* 
        Initializing the lines that are moving
     */
    @FXML
    private Line addAbFixedLine;
    @FXML
    private Line addAbDyLine;
    @FXML
    private Line intExtFixedLine;
    @FXML
    private Line intExtDyLine;

    @FXML
    private Line kneeFlexFixedLine;
    @FXML
    private Line kneeFlexDyLine;

    @FXML
    private Line hipFlexFixedLine;
    @FXML
    private Line hipFlexDyLine;

    // vertical box to hold open ports. 
    @FXML
    private VBox ports;

    @Override
    public void initialize(URL url, ResourceBundle rb) {
        // TODo

        initializePositions();

        detectPort();

        final ComboBox comboBoxPorts = new ComboBox(portList);
        comboBoxPorts.valueProperty()
                .addListener(new ChangeListener<String>() {

                    @Override
                    public void changed(ObservableValue<? extends String> observable,
                            String oldValue, String newValue) {

                        System.out.println(newValue);
                        disconnectArduino();
                        connectArduino(newValue);
                    }

                });

        ports.getChildren().addAll(comboBoxPorts);
    }

    // gets the list of open COM ports on the computer. 
    private void detectPort() {

        portList = FXCollections.observableArrayList();

        String[] serialPortNames = SerialPortList.getPortNames();
        for (String name : serialPortNames) {
            System.out.println(name);
            portList.add(name);
        }
    }

    public boolean connectArduino(String port) {

        System.out.println("connectArduino");

        boolean success = false;
        SerialPort serialPort = new SerialPort(port);
        try {
            serialPort.openPort();
            serialPort.setParams(
                    SerialPort.BAUDRATE_9600,
                    SerialPort.DATABITS_8,
                    SerialPort.STOPBITS_1,
                    SerialPort.PARITY_NONE);
            serialPort.setEventsMask(MASK_RXCHAR);
            serialPort.addEventListener((SerialPortEvent serialPortEvent) -> {
                if (serialPortEvent.isRXCHAR() && serialPortEvent.getEventValue() > 0) {
                    try {
                        /* 
                            Reads in every byte sent in the serial port. 
                            If a \n is found, then package that string and 
                            convert it to the int to make to angle. If a comma 
                            is found, up the count and append the to the next 
                            angle. Endless stream until the application is shut 
                            down. 
                         */
                        byte buffer[] = serialPort.readBytes(serialPortEvent.getEventValue());
                        for (byte b : buffer) {
                            if (b == 44) {
                                count++;
                            } else if (b == 10) {
                                count = 0;
                                String angle1Process = angle1.toString();
                                String angle2Process = angle2.toString();
                                String angle3Process = angle3.toString();
                                String angle4Process = angle4.toString();

                                //toProcess.split(" ");
                                String temp1 = new String();
                                temp1 = angle1Process.trim();

                                String temp2 = new String();
                                temp2 = angle2Process.trim();

                                // to be used
                                String temp3 = new String();
                                temp3 = angle3Process.trim();

                                // to be used
                                String temp4 = new String();
                                temp4 = angle4Process.trim();

                                System.out.println("Angle1: " + angle1Process);
                                System.out.println("Angle2: " + angle2Process);
                                System.out.println("Angle3: " + angle3Process);
                                System.out.println("Angle4: " + angle4Process);
                                try {
                                    angle1.setLength(0);
                                    angle2.setLength(0);
                                    angle3.setLength(0); // to be used
                                    angle4.setLength(0); // to be used

                                    // first angle calculations
                                    Integer x = Integer.parseInt(temp1);
                                    Platform.runLater(() -> {
                                        intExtLabel.setText("Interior/Ext Angle: " + x);
                                    });
                                    intExtDyLine.setEndX(200 * Math.sin((x - 180) * Math.PI / 180));
                                    intExtDyLine.setEndY(200 * Math.cos((x - 180) * Math.PI / 180));

                                    // second angle calculations
                                    Integer y = Integer.parseInt(temp2);
                                    Platform.runLater(() -> {
                                        hipAbAdLabel.setText("Hip Adduction.Abduction Angle: " + y);
                                    });
                                    addAbDyLine.setEndX(200 * Math.sin((360 - y) * Math.PI / 180));
                                    addAbDyLine.setEndY(200 * Math.cos((360 - y) * Math.PI / 180));

                                    Integer z = Integer.parseInt(temp3);
                                    Platform.runLater(() -> {
                                        kneeFlexExtenLabel.setText("Knee Flexion/Extension Angle: " + z);
                                    });
                                    kneeFlexDyLine.setEndX(200 * Math.sin((z - 45) * Math.PI / 180));
                                    kneeFlexDyLine.setEndY(200 * Math.cos((z - 45) * Math.PI / 180));

                                    Integer a = Integer.parseInt(temp4);
                                    Platform.runLater(() -> {
                                        hipFlexExtenLabel.setText("Hip Flexion/Extension Angle: " + a);
                                    });
                                    hipFlexDyLine.setEndX(200 * Math.sin((a - 270) * Math.PI / 180));
                                    hipFlexDyLine.setEndY(200 * Math.cos((a - 270) * Math.PI / 180));
                                    
                                    rxBox.setAngle(0);
                                    ryBox.setAngle(x);
                                    rzBox.setAngle(0);
                                    box.getTransforms().addAll(rxBox, ryBox, rzBox);

                                } catch (NumberFormatException e) {
                                    e.printStackTrace();
                                }

                            } else {
                                if (count == 0) {
                                    angle1.append((char) b);
                                    //System.out.println(angle1);
                                } else if (count == 1) {
                                    angle2.append((char) b);
                                } else if (count == 2) {
                                    angle3.append((char) b);
                                } else if (count == 3) {
                                    angle4.append((char) b);
                                }
                                //System.out.println("appending");
                                //System.out.println(sb);

                            }
                        }

                        //Update label in ui thread
                    } catch (SerialPortException ex) {
                        Logger.getLogger(FXMLDocumentController.class.getName())
                                .log(Level.SEVERE, null, ex);
                    } catch (NumberFormatException e) {

                    }
                }
            });

            arduinoPort = serialPort;
            success = true;
        } catch (SerialPortException ex) {
            Logger.getLogger(FXMLDocumentController.class.getName())
                    .log(Level.SEVERE, null, ex);
            System.out.println("SerialPortException: " + ex.toString());
        }

        return success;
    }

    // gets the open port and closes it. 
    public void disconnectArduino() {

        System.out.println("disconnectArduino()");
        if (arduinoPort != null) {
            try {
                arduinoPort.removeEventListener();

                if (arduinoPort.isOpened()) {
                    arduinoPort.closePort();
                }

            } catch (SerialPortException ex) {
                Logger.getLogger(FXMLDocumentController.class.getName())
                        .log(Level.SEVERE, null, ex);
            }
        }
    }

    public void stop() throws Exception {
        disconnectArduino();
        //stop();
    }

    public void initializePositions() {
        // initializing the view setup
        AdAbButton.setSelected(true);
        abAdView.setVisible(true);
        hipFlexExtenView.setVisible(false);
        intExtView.setVisible(false);
        kneeFlexExtenView.setVisible(false);

        /*
           adduction/abduction view
         */
        addAbFixedLine.setTranslateX(300);
        addAbFixedLine.setTranslateY(150);
        addAbFixedLine.setStartX(0);
        addAbFixedLine.setStartY(0);
        addAbFixedLine.setEndX(0);
        addAbFixedLine.setEndY(200);

        addAbDyLine.setTranslateX(300);
        addAbDyLine.setTranslateY(150);
        addAbDyLine.setStartX(0);
        addAbDyLine.setStartY(0);
        addAbDyLine.setEndX(200 * Math.sin((90 + 180) * Math.PI / 180));
        addAbDyLine.setEndY(200 * Math.cos((90 + 180) * Math.PI / 180));

        /*
           internal/external view
         */
        intExtFixedLine.setTranslateX(300);
        intExtFixedLine.setTranslateY(150);
        intExtFixedLine.setStartX(0);
        intExtFixedLine.setStartY(0);
        intExtFixedLine.setEndX(0);
        intExtFixedLine.setEndY(200);

        intExtDyLine.setTranslateX(300);
        intExtDyLine.setTranslateY(350);
        intExtDyLine.setStartX(0);
        intExtDyLine.setStartY(0);
        intExtDyLine.setEndX(200 * Math.sin((360 - 90) * Math.PI / 180));
        intExtDyLine.setEndY(200 * Math.cos((360 - 90) * Math.PI / 180));

        // knee View
        kneeFlexFixedLine.setTranslateX(300);
        kneeFlexFixedLine.setTranslateY(150);
        kneeFlexFixedLine.setStartX(0);
        kneeFlexFixedLine.setStartY(0);
        kneeFlexFixedLine.setEndX(-150);
        kneeFlexFixedLine.setEndY(150);

        kneeFlexDyLine.setTranslateX(300);
        kneeFlexDyLine.setTranslateY(150);
        kneeFlexDyLine.setStartX(0);
        kneeFlexDyLine.setStartY(0);
        kneeFlexDyLine.setEndX(200 * Math.sin((90 - 45) * Math.PI / 180));
        kneeFlexDyLine.setEndY(200 * Math.cos((90 - 45) * Math.PI / 180));

        // hip view
        hipFlexFixedLine.setTranslateX(300);
        hipFlexFixedLine.setTranslateY(300);
        hipFlexFixedLine.setStartX(0);
        hipFlexFixedLine.setStartY(0);
        hipFlexFixedLine.setEndX(200);
        hipFlexFixedLine.setEndY(0);

        hipFlexDyLine.setTranslateX(300);
        hipFlexDyLine.setTranslateY(300);
        hipFlexDyLine.setStartX(0);
        hipFlexDyLine.setStartY(0);
        hipFlexDyLine.setEndX(200 * Math.sin((270 - 90) * Math.PI / 180));
        hipFlexDyLine.setEndY(200 * Math.cos((270 - 90) * Math.PI / 180));
    }
}
