/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package hip.positioning.system;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.shape.Line;
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
    int hipFlexExtenAngle = 90;
    int kneeFlexExtenAngle = 90;
    int intExtAngle = 90;
    int adAbAngle = 90;
    
    @FXML 
    private Pane flexExtenView;
    
    @FXML 
    private Pane intExtView;
    
    @FXML 
    private Pane abAdView;
    
    @FXML 
    private Label intExtLabel;
    
    @FXML
    private Label kneeFlexExtenLabel;
    
    @FXML
    private Label hipFlexExtenLabel;
    
    @FXML
    private Label hipAbAdLabel;
    
    @FXML
    private RadioButton AdAbButton;
    
    @FXML
    private RadioButton intExtButton;
    
    @FXML
    private RadioButton flexExtButton;
 
    @FXML
    private Button btn2;
    
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
        } 
        catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /*
        These methods are for decrementing angles and incrementing angles
    */
    @FXML
    private void increaseAngle(ActionEvent event) {
        if (AdAbButton.isSelected()) {
           adAbAngle++;
           hipAbAdLabel.setText("Hip Adduction/Abduction Angle: " + adAbAngle);
           addAbDyLine.setEndX(200 * Math.sin((360 - adAbAngle) * Math.PI / 180));
           addAbDyLine.setEndY(200 * Math.cos((360 - adAbAngle) * Math.PI / 180));
        }
        if (intExtButton.isSelected()) {
           intExtAngle++;
           intExtLabel.setText("Hip Internal/External Angle: " + intExtAngle);
           intExtDyLine.setEndX(200 * Math.sin((intExtAngle - 180) * Math.PI / 180));
           intExtDyLine.setEndY(200 * Math.cos((intExtAngle - 180) * Math.PI / 180));
        }
    }
    
    @FXML 
    private void decreaseAngle(ActionEvent event) {
        if (AdAbButton.isSelected()) {
           adAbAngle--;
           hipAbAdLabel.setText("Hip Adduction/Abduction Angle: " + adAbAngle);
           addAbDyLine.setEndX(200 * Math.sin((360 - adAbAngle) * Math.PI / 180));
           addAbDyLine.setEndY(200 * Math.cos((360 - adAbAngle) * Math.PI / 180));
        }
        if (intExtButton.isSelected()) {
           intExtAngle--;
           intExtLabel.setText("Hip Internal/External Angle: " + intExtAngle);
           intExtDyLine.setEndX(200 * Math.sin((intExtAngle - 180) * Math.PI / 180));
           intExtDyLine.setEndY(200 * Math.cos((intExtAngle - 180) * Math.PI / 180));
        }
    }
    
    /*
        These methods are for changing the viewing angles
    */
    @FXML 
    private void changeToAddAb(ActionEvent event) {
        abAdView.setVisible(true);
        flexExtenView.setVisible(false);
        intExtView.setVisible(false);
    }
    
    @FXML 
    private void changeToFlexExten(ActionEvent event) {
        abAdView.setVisible(false);
        flexExtenView.setVisible(true);
        intExtView.setVisible(false);
    }
    
    @FXML 
    private void changeToIntExt(ActionEvent event) {
        abAdView.setVisible(false);
        flexExtenView.setVisible(false);
        intExtView.setVisible(true);
    }
    
    /* 
        Initializing the lines
    */
    @FXML 
    private Line addAbFixedLine;
    @FXML
    private Line addAbDyLine;
    @FXML 
    private Line intExtFixedLine;
    @FXML
    private Line intExtDyLine;
    
    @Override
    public void initialize(URL url, ResourceBundle rb) {
        // TODO
        intExtLabel.setText("Hip Internal/External Angle: " + intExtAngle);
        kneeFlexExtenLabel.setText("Knee Flexion/Extension Angle: " + kneeFlexExtenAngle);
        hipFlexExtenLabel.setText("Hip Flex/Extension Angle: " + hipFlexExtenAngle);
        hipAbAdLabel.setText("Hip Adduction/Abduction Angle: " + adAbAngle);
        
        // initializing the view setup
        AdAbButton.setSelected(true);
        abAdView.setVisible(true);
        flexExtenView.setVisible(false);
        intExtView.setVisible(false);
        
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
        addAbDyLine.setEndX(200 * Math.sin((adAbAngle + 180) * Math.PI / 180));
        addAbDyLine.setEndY(200 * Math.cos((adAbAngle + 180) * Math.PI / 180));
        
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
        intExtDyLine.setEndX(200 * Math.sin((360 - intExtAngle) * Math.PI / 180));
        intExtDyLine.setEndY(200 * Math.cos((360 - intExtAngle) * Math.PI / 180));
    }    
    
}
