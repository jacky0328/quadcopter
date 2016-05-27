package plotPID;

//public class PID {

	
	
	
//}
import javax.swing.*;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;
import javax.swing.filechooser.FileFilter;



import java.awt.*;
import java.awt.event.*;

import java.io.*;
import java.security.AccessControlException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.awt.*; 
import java.awt.RenderingHints;
import java.awt.Font;

import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

public class PID extends JComponent implements ActionListener ,MouseListener  {
	
	
	FileInputStream in ;
	BufferedReader br;
	int ESC0_data [];
	int ESC1_data [];
	int ESC2_data [];
	int ESC3_data [];
	float PITCH_data[];
	float ROLL_data[];
	float PITCH_gdata[];
	float ROLL_gdata[];
	int PIDOut_PITCH_P[];
	int PIDOut_PITCH_I[];
	int PIDOut_PITCH_D[];
	int PIDOut_PITCH[];
	int PIDOut_ROLL_P[];
	int PIDOut_ROLL_I[];
	int PIDOut_ROLL_D[];
	int PIDOut_ROLL[];
	
	int line_cnt =0;
	
	int input_scale=1;
	int input_offset=0;
	private JTextField mInputScale;
	private JLabel     mLInputscale;
	private JTextField mInputOffset;
	private JLabel     mLInputOffset;
	
	private JLabel     m_FL_PWM;
	private JLabel     m_FR_PWM;
	private JLabel     m_BL_PWM;
	private JLabel     m_BR_PWM;
	private JLabel     m_PITCH;
	private JLabel     m_ROLL;
	private JLabel     m_ROLL_PID_SUM;
	private JLabel     m_PITCH_PID_SUM;
	private JLabel     m_Xcoord;
	
	
	private JCheckBox  mchbox_FL;
	private JCheckBox  mchbox_FR;
	private JCheckBox  mchbox_BL;
	private JCheckBox  mchbox_BR;
	private JCheckBox  mchbox_G_PITCH;
	private JCheckBox  mchbox_G_ROLL;
	private boolean FL_EN=true;
	private boolean FR_EN=true;
	private boolean BL_EN=true;
	private boolean BR_EN=true;
	private boolean GP_EN=true;
	private boolean GR_EN=true;
	
	private JButton     moffsetAdd;
	private JButton     moffsetSub;
	
	private int m_x;
	private int m_y;
	
	private int m_lx=0;
	private int m_ly=0;
	private int m_rx=0;
	private int m_ry=0;
	
	private int m_coord_x=0;
	
	private  String path;
	
	private  int   para_offset = 630;
	  private File mSelectedProject;
	 final JFileChooser mFileChooser;

	    {
	        JFileChooser fc = new JFileChooser();
	        try
	        {
	        fc.setDialogTitle("Open file");
	        fc.setFileFilter(new FileFilter() {
	            final Matcher matchLevelFile = Pattern.compile(".*\\.log[z]?").matcher("");
	            public boolean accept(File file)
	            {
	                if (file.isDirectory()) return true;
	                matchLevelFile.reset(file.getName());
	                return matchLevelFile.matches();
	            }

	            public String getDescription() { return "log file (*.log)"; }
	        });
	        } catch (AccessControlException ex) {
	            //Do not create file chooser if webstart refuses permissions
	        }
	        mFileChooser = fc;
	    }
	
	    private void openFileChooser() {
	        if (mFileChooser == null) return;
	        int retValue = mFileChooser.showOpenDialog(null);
	        if (retValue == JFileChooser.APPROVE_OPTION) {
	            mSelectedProject = mFileChooser.getSelectedFile();
	            
	            if (mSelectedProject == null) {
	                return;
	            }

	          
	            try {
	                path = mSelectedProject.getCanonicalPath();
	            } catch (IOException ex) {
	            
	                path = mSelectedProject.getName();
	            }
	            System.out.println("Path:" + path);
	            
	                     
	        } else {
	            System.out.println("No Selection ");
	        }
	    }
	    
 public PID() {

	 openFileChooser();
	    
     //add mouse listener
     addMouseListener(this);
	 
	 
	 try {
		 in = new FileInputStream(path);
		 br = new BufferedReader(new InputStreamReader(in));
		 
		 
		 
		 String words[]=new String[20];
		 String line=null;
		
		 
		 while((line=br.readLine())!=null){
	            line_cnt++;
	     }
		    br.close();
		 		 }
		 catch(Exception  e){
			  e.printStackTrace();
			 
		 }
	 
	 System.out.println("line_cnt : " + line_cnt);
	 ESC0_data = new int[line_cnt];
	 ESC1_data = new int[line_cnt];
	 ESC2_data = new int[line_cnt];
	 ESC3_data = new int[line_cnt];
	 PITCH_data = new float[line_cnt];
	 ROLL_data  = new float[line_cnt];
	 PIDOut_PITCH_P = new int[line_cnt];
	 PIDOut_PITCH_I = new int[line_cnt];
	 PIDOut_PITCH_D = new int[line_cnt];
	 PIDOut_PITCH  = new int[line_cnt];
	 PIDOut_ROLL_P = new int[line_cnt];
	 PIDOut_ROLL_I = new int[line_cnt];
	 PIDOut_ROLL_D = new int[line_cnt];
	 PIDOut_ROLL   = new int[line_cnt];
	 PITCH_gdata = new float[line_cnt];
	 ROLL_gdata  = new float[line_cnt];
	 
	 try {
		 in = new FileInputStream(path);
		 br = new BufferedReader(new InputStreamReader(in));
		 
		 String words[]=new String[21];
		 String line=null;
	     int index =0;
		 while((line=br.readLine())!=null){
	         words=line.split(" ");
	         int RC_0=Integer.parseInt(words[0]);
	         int RC_1=Integer.parseInt(words[1]);
	         int RC_2=Integer.parseInt(words[2]);
	         int RC_3=Integer.parseInt(words[3]);
	         float YAW = Float.parseFloat(words[4]);
	         float PITCH = Float.parseFloat(words[5]);
	         float ROLL = Float.parseFloat(words[6]);
	         int ESC0 = Integer.parseInt(words[7]);
	         int ESC1 = Integer.parseInt(words[8]);
	         int ESC2 = Integer.parseInt(words[9]);
	         int ESC3 = Integer.parseInt(words[10]);
	        
	         
	         int PITCH_P = (int)Float.parseFloat(words[11]);
	         int PITCH_I = (int)Float.parseFloat(words[12]);
	         int PITCH_D = (int)Float.parseFloat(words[13]);
	         int PITCH_SUM = (int)Float.parseFloat(words[14]);
	         int ROLL_P = (int)Float.parseFloat(words[15]);
	         int ROLL_I = (int)Float.parseFloat(words[16]);
	         int ROLL_D = (int)Float.parseFloat(words[17]);
	         int ROLL_SUM = (int)Float.parseFloat(words[18]);
	         float PITCH_G = Float.parseFloat(words[19]);
	         float ROLL_G = Float.parseFloat(words[20]);
	         
	         
	       //  System.out.println(String.format("RC_0 : %d, RC_1 : %d, RC_2 : %d, RC_3 : %d, Y : %f, P : %f, R : %f, ESC0 : %d, ESC1 : %d, ESC2 : %d, ESC3 : %d ",
	       // 		 RC_0,RC_1,RC_2,RC_3,YAW,PITCH,ROLL,ESC0,ESC1,ESC2,ESC3));
	         //System.out.println(PITCH);
	         PITCH_data[index] = PITCH;
	         ROLL_data[index] = ROLL;
	         ESC0_data[index] = ESC0; // FL
	         ESC1_data[index] = ESC1; // FR
	         ESC2_data[index] = ESC2; // BL
	         ESC3_data[index] = ESC3; // BR
	         PIDOut_PITCH_P[index] = PITCH_P; // pitch p_sum
	         PIDOut_PITCH_I[index] = PITCH_I; // pitch i_sum
	         PIDOut_PITCH_D[index] = PITCH_D; // pitch d_sum
	         PIDOut_PITCH[index]   = PITCH_SUM; // pitch sum
	         PIDOut_ROLL_P[index] = ROLL_P;   // roll p_sum
	         PIDOut_ROLL_I[index] = ROLL_I;   // roll i_sum
	         PIDOut_ROLL_D[index] = ROLL_D;   // roll d_sum
	         PIDOut_ROLL[index]   = ROLL_SUM;   // roll d_sum
	         PITCH_gdata[index]  = PITCH_G;
	         ROLL_gdata[index]  = ROLL_G;
	         index++;
	     }
		    br.close();
		    
		 
		 }
		 catch(Exception  e){
			  e.printStackTrace();
			 
		 }
  }
  
 public void mouseClicked(MouseEvent e) {
     
    
	// m_x = e.getX();
//	 m_y = e.getY();
     repaint();
}


public void mousePressed(MouseEvent e) {
     
     m_x = e.getX();
     m_y = e.getY();
    
     switch(e.getModifiers()) 
     {
     case InputEvent.BUTTON1_MASK: {
       //System.out.println("That's the LEFT button");  
    	 m_lx = m_x;
    	 m_ly = m_y;
       break;
       }
     case InputEvent.BUTTON2_MASK: {
       System.out.println("That's the MIDDLE button");     
       break;
       }
     case InputEvent.BUTTON3_MASK: {
       //System.out.println("That's the RIGHT button");   
    	 m_rx = m_x;
    	 m_ry = m_y;
       break;
       }
     }
   
     m_coord_x = input_offset +  m_x;
   
     System.out.println(ESC0_data[m_coord_x]);
     
     m_FL_PWM.setText("FL:"+Integer.toString(ESC0_data[m_coord_x]));
     m_FR_PWM.setText("FR:"+Integer.toString(ESC1_data[m_coord_x]));
     m_BL_PWM.setText("BL:"+Integer.toString(ESC2_data[m_coord_x]));
     m_BR_PWM.setText("BR:"+Integer.toString(ESC3_data[m_coord_x]));
     m_PITCH.setText("PITCH:"+Integer.toString((int)PITCH_data[m_coord_x]));
     m_ROLL.setText("ROLL:"+Integer.toString((int)ROLL_data[m_coord_x]));
     
     m_PITCH_PID_SUM.setText("P_PID_SUM:"+Integer.toString((int)PIDOut_PITCH[m_coord_x]));
     m_ROLL_PID_SUM.setText ("R_PID_SUM:"+Integer.toString((int)PIDOut_ROLL[m_coord_x]));
     m_Xcoord.setText ("X_COORD:"+Integer.toString((int)m_coord_x));
     repaint();

}


public void mouseReleased(MouseEvent e) {
     
   //  m_x = e.getX();
   //  m_y = e.getY();
   //  repaint();

}


public void mouseEntered(MouseEvent e) {
     
  //   m_x = e.getX();
  //   m_y = e.getY();
  //   repaint();

}


public void mouseExited(MouseEvent e) {
     
   //  m_x = e.getX();
   //  m_y = e.getY();
   //  repaint();

}
 
 public void paintComponent(Graphics g) {
  super.paintComponent(g);

  //畫一條直線,從座標(100,100)到(150,150)
 // g.drawLine(100,100,150,150);
  
  int   pitch_roll_sensor_offset =580;
  int   PID_offset = 500;
 
  
  int i,j;
  int scale=input_scale;
  int offset_0=1000;
  int sum0,sum1;
  
	  
  g.setColor(Color.RED);
  
  if(FL_EN==true){
	  
	/*  
  if(scale>=1){
	  int  scale_inv = 1/ scale;
	  
	  for(i=input_offset;i<line_cnt-scale*2;i++){
		 
			  sum0 =  ESC0_data[(i-input_offset)/2 + input_offset];
			  sum1 = ESC0_data[(i-input_offset)/2+1 + input_offset];
		  g.drawLine(i-input_offset,1500-sum0,i-input_offset+1,1500-sum1);
	  }
	  
  }else{
	  */
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		  for(j=0;j<scale;j++){
			  sum0 +=  ESC0_data[i+j];
			  sum1 += ESC0_data[i+j+scale];
		  }
		  sum0 = sum0/scale;
		  sum1 = sum1/scale;
		  g.drawLine((i-input_offset)/scale,1500-sum0,(i-input_offset)/scale+1,1500-sum1);
	  }
  }
 // }
  
  g.setColor(Color.YELLOW);
  if(FR_EN==true){
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 +=  ESC1_data[i+j];
	        sum1 += ESC1_data[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,1500-sum0,(i-input_offset)/scale+1,1500-sum1);
	  }
	  }
  
  g.setColor(Color.PINK);
  
  if(BL_EN==true){
  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
	  sum0=0;
	  sum1=0;
	 for(j=0;j<scale;j++){
        sum0 += ESC2_data[i+j];
        sum1 += ESC2_data[i+j+scale];
	 }
	 sum0 = sum0/scale;
	 sum1 = sum1/scale;
    g.drawLine((i-input_offset)/scale,1500-sum0,(i-input_offset)/scale+1,1500-sum1);
  }
  }
      
  
  
	  g.setColor(Color.BLUE);
    
	  if(BR_EN==true){
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 +=  ESC3_data[i+j];
	        sum1 += ESC3_data[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,1500-sum0,(i-input_offset)/scale+1,1500-sum1);
	  }
	  }
	  
	  g.setColor(Color.ORANGE);
	
	 
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 +=  PITCH_data[i+j];
	        sum1 += PITCH_data[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,pitch_roll_sensor_offset-sum0,(i-input_offset)/scale+1,pitch_roll_sensor_offset-sum1);
	  }
	  
	  
	  g.setColor(Color.GREEN);
	 
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 +=  ROLL_data[i+j];
	        sum1 += ROLL_data[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,pitch_roll_sensor_offset-sum0,(i-input_offset)/scale+1,pitch_roll_sensor_offset-sum1);
	  }
	  
	  if(GP_EN==true){
	  g.setColor(Color.RED);
		
		 
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 +=  PITCH_gdata[i+j];
	        sum1 += PITCH_gdata[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,pitch_roll_sensor_offset-sum0,(i-input_offset)/scale+1,pitch_roll_sensor_offset-sum1);
	  }
	  }
	  
	  if(GR_EN==true){
	  g.setColor(Color.BLUE);
		 
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 +=  ROLL_gdata[i+j];
	        sum1 += ROLL_gdata[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,pitch_roll_sensor_offset-sum0,(i-input_offset)/scale+1,pitch_roll_sensor_offset-sum1);
	  }
	  }
	  //------------------------------------------------------
	  
	  g.setColor(Color.RED);
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 += PIDOut_ROLL[i+j];
	        sum1 += PIDOut_ROLL[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,PID_offset-sum0,(i-input_offset)/scale+1,PID_offset-sum1);
	  }
	  
	  g.setColor(Color.BLUE);
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 += PIDOut_ROLL_P[i+j];
	        sum1 += PIDOut_ROLL_P[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,PID_offset-sum0,(i-input_offset)/scale+1,PID_offset-sum1);
	  }
	  
	  g.setColor(Color.YELLOW);
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 += PIDOut_ROLL_I[i+j];
	        sum1 += PIDOut_ROLL_I[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,PID_offset-sum0,(i-input_offset)/scale+1,PID_offset-sum1);
	  }
	  
	  g.setColor(Color.PINK);
	  for(i=input_offset;i<line_cnt-scale*2;i+=scale){
		  sum0=0;
		  sum1=0;
		 for(j=0;j<scale;j++){
	        sum0 += PIDOut_ROLL_D[i+j];
	        sum1 += PIDOut_ROLL_D[i+j+scale];
		 }
		 sum0 = sum0/scale;
		 sum1 = sum1/scale;
	    g.drawLine((i-input_offset)/scale,PID_offset-sum0,(i-input_offset)/scale+1,PID_offset-sum1);
	  }
	  
	  
	  
	  
	  
	  g.setColor(Color.WHITE);
      Font font = new Font("Serif", Font.PLAIN, 10);
      g.setFont(font);
	 
      g.drawLine(0,400,1279,400);
	  g.drawString("PWM 1100", 0, 410);
	  
	  g.drawLine(0,350,1279,350);
	  g.drawString("PWM 1150", 0, 360);
	  
	  g.drawLine(0,300,1279,300);
	  g.drawString("PWM 1200", 0, 310);
	  
	  g.drawLine(0,250,1279,250);
	  g.drawString("PWM 1250", 0, 260);
	  
	  g.drawLine(0,200,1279,200);
	  g.drawString("PWM 1300", 0, 210);
	  
	  g.drawLine(0,150,1279,150);
	  g.drawString("PWM 1350", 0, 160);
	  
	  g.drawLine(0,100,1279,100);
	  g.drawString("PWM 1400", 0, 110);
	  
	  g.drawLine(0,50,1279,50);
	  g.drawString("PWM 1450", 0, 60);
	  
	  g.drawLine(0,00,1279,00);
	  g.drawString("PWM 1500", 0, 10);
	      
	  
	  g.drawLine(0,pitch_roll_sensor_offset,1279,pitch_roll_sensor_offset);
	  g.drawString("0 angle", 0, pitch_roll_sensor_offset+20);
	  
	  g.drawLine(0,pitch_roll_sensor_offset-30,1279,pitch_roll_sensor_offset-30);
	  g.drawString("30 angle", 0, pitch_roll_sensor_offset-30+20);
	  	  
	  g.setColor(Color.RED);
	  g.drawString("RED  : FL",  0, para_offset);
	  g.setColor(Color.YELLOW);
	  g.drawString("YELLOW : FR",  0, para_offset+10);
	  g.setColor(Color.PINK);
	  g.drawString("PINK : BL", 0, para_offset+20);
	  g.setColor(Color.BLUE);
	  g.drawString("BLUE : BR", 0, para_offset+30);
	  g.setColor(Color.ORANGE);
	  g.drawString("ORANGE : PITCH", 0, para_offset+40);
	  g.setColor(Color.GREEN);
	  g.drawString("GREEN : ROLL", 0, para_offset+50);
	  
	  g.setColor(Color.RED);
	  g.drawString("RED    : ROLL_PID_SUM",  90, para_offset);
	  g.setColor(Color.BLUE);
	  g.drawString("BLUE   : ROLL_PID_P",  90, para_offset+10);	  
	  g.setColor(Color.YELLOW);
	  g.drawString("YELLOW : ROLL_PID_I",  90, para_offset+20);	  
	  g.setColor(Color.PINK);
	  g.drawString("PINK   : ROLL_PID_D",  90, para_offset+30);	  
	  
	  g.drawLine(m_lx,0,m_lx,719);
	  g.drawLine(m_rx,0,m_rx,719);
	  
	  
	  
 }

 //實做自訂元件時，
 //最好覆寫這getPreferredSize、getMaximumSize、getMinimumSize三個方法
 //因為許多LayoutManager都可能由這三個方法來決定此物件的大小
 public Dimension getPreferredSize() {
  return new Dimension(1280, 720);
 }

 public Dimension getMaximumSize() {
  return getPreferredSize();
 }

 public Dimension getMinimumSize() {
  return getPreferredSize();
 }
 
 public void input_scale_init() {
	 
	 
	 int  out_string_ofset = 700;
	 int  out_string_ofset2 = 250;  
	  
	
	  mLInputscale = new JLabel("Scale : ");
	  mLInputscale.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mLInputscale.setForeground(Color.WHITE);
	  mLInputscale.setBounds(out_string_ofset2+300, para_offset, 50, 30);	
	  
	  mInputScale = new JTextField("1");
	  mInputScale.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mInputScale.setColumns(10);
	  mInputScale.setBounds(out_string_ofset2+350, para_offset, 30, 30);	 
	  mInputScale.setActionCommand("input_scale");
	  mInputScale.addActionListener(this);
	  
	   
	  mInputOffset = new JTextField("0");
	  mInputOffset.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mInputOffset.setColumns(10);
	  mInputOffset.setBounds(out_string_ofset2+40, para_offset, 60, 30);	 
	  mInputOffset.setActionCommand("input_scale");
	  mInputOffset.addActionListener(this);
	  
	  mLInputOffset = new JLabel("OFFSET: ");
	  mLInputOffset.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mLInputOffset.setForeground(Color.WHITE);
	  mLInputOffset.setBounds(out_string_ofset2, para_offset, 50, 30);	
	  
	  moffsetAdd = new JButton("+ 200 ");
	  moffsetAdd.setFont(new Font("Monaco", Font.PLAIN, 14));
	  moffsetAdd.setBackground(Color.WHITE);
	  moffsetAdd.setBounds(out_string_ofset2+100, para_offset, 80, 30);
	  moffsetAdd.setActionCommand("offset_add");
	  moffsetAdd.addActionListener(this);
	  
	  moffsetSub = new JButton("- 200 ");
	  moffsetSub.setFont(new Font("Monaco", Font.PLAIN, 14));
	  moffsetSub.setBackground(Color.WHITE);
	  moffsetSub.setBounds(out_string_ofset2+200, para_offset, 80, 30);
	  moffsetSub.setActionCommand("offset_sub");
	  moffsetSub.addActionListener(this);
	  
	  mchbox_FL = new  JCheckBox("FL OUTPT");
	  mchbox_FL.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mchbox_FL.setSelected(true);
	  mchbox_FL.setVisible(true);
	  mchbox_FL.setBounds(out_string_ofset2, para_offset+30, 100, 20);
	  mchbox_FL.addActionListener(this);
	  
	  mchbox_FR = new  JCheckBox("FR OUTPT");
	  mchbox_FR.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mchbox_FR.setSelected(true);
	  mchbox_FR.setVisible(true);
	  mchbox_FR.setBounds(out_string_ofset2+100, para_offset+30, 100, 20);
	  mchbox_FR.addActionListener(this);
	  
	  mchbox_BL = new  JCheckBox("BL OUTPT");
	  mchbox_BL.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mchbox_BL.setSelected(true);
	  mchbox_BL.setVisible(true);
	  mchbox_BL.setBounds(out_string_ofset2+200, para_offset+30, 100, 20);
	  mchbox_BL.addActionListener(this);
	  
	  mchbox_BR = new  JCheckBox("BR OUTPT");
	  mchbox_BR.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mchbox_BR.setSelected(true);
	  mchbox_BR.setVisible(true);
	  mchbox_BR.setBounds(out_string_ofset2+300, para_offset+30, 100, 20);
	  mchbox_BR.addActionListener(this);
	  
	  mchbox_G_PITCH = new  JCheckBox("G_PITCH OUTPT");
	  mchbox_G_PITCH.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mchbox_G_PITCH.setSelected(true);
	  mchbox_G_PITCH.setVisible(true);
	  mchbox_G_PITCH.setBounds(out_string_ofset2+400, para_offset+30, 100, 20);
	  mchbox_G_PITCH.addActionListener(this);
	  
	  mchbox_G_ROLL = new  JCheckBox("G_PITCH OUTPT");
	  mchbox_G_ROLL.setFont(new Font("Monaco", Font.PLAIN, 14));
	  mchbox_G_ROLL.setSelected(true);
	  mchbox_G_ROLL.setVisible(true);
	  mchbox_G_ROLL.setBounds(out_string_ofset2+500, para_offset+30, 100, 20);
	  mchbox_G_ROLL.addActionListener(this);
	  
	  // mchbox_G_PITCH
	  
	  m_FL_PWM = new JLabel("0");
	  m_FL_PWM.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_FL_PWM.setForeground(Color.PINK);
	  m_FL_PWM.setBounds(out_string_ofset, para_offset, 70, 20);	
	  
	  m_FR_PWM = new JLabel("0");
	  m_FR_PWM.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_FR_PWM.setForeground(Color.PINK);
	  m_FR_PWM.setBounds(out_string_ofset+70, para_offset, 70, 20);	
	  
	  m_BL_PWM = new JLabel("0");
	  m_BL_PWM.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_BL_PWM.setForeground(Color.PINK);
	  m_BL_PWM.setBounds(out_string_ofset+70*2, para_offset, 70, 20);	
	  
	  m_BR_PWM = new JLabel("0");
	  m_BR_PWM.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_BR_PWM.setForeground(Color.PINK);
	  m_BR_PWM.setBounds(out_string_ofset+70*3, para_offset, 70, 20);	
	  
	  m_PITCH = new JLabel("0");
	  m_PITCH.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_PITCH.setForeground(Color.PINK);
	  m_PITCH.setBounds(out_string_ofset+70*4, para_offset, 70, 20);	
	  
	  m_ROLL = new JLabel("0");
	  m_ROLL.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_ROLL.setForeground(Color.PINK);
	  m_ROLL.setBounds(out_string_ofset+70*5, para_offset, 70, 20);	
	  
	  m_PITCH_PID_SUM = new JLabel("0");
	  m_PITCH_PID_SUM.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_PITCH_PID_SUM.setForeground(Color.PINK);
	  m_PITCH_PID_SUM.setBounds(out_string_ofset, para_offset+20, 120, 20);	
	  
	  m_ROLL_PID_SUM = new JLabel("0");
	  m_ROLL_PID_SUM.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_ROLL_PID_SUM.setForeground(Color.PINK);
	  m_ROLL_PID_SUM.setBounds(out_string_ofset+120, para_offset+20, 120, 20);	
	  
	  m_Xcoord = new JLabel("0");
	  m_Xcoord.setFont(new Font("Monaco", Font.PLAIN, 14));
	  m_Xcoord.setForeground(Color.PINK);
	  m_Xcoord.setBounds(out_string_ofset+240, para_offset+20, 120, 20);	
  }
 
 public JLabel get_flpwm_label(){
	 return  m_FL_PWM;
  }
 public JLabel get_frpwm_label(){
	 return  m_FR_PWM;
  }
 public JLabel get_blpwm_label(){
	 return  m_BL_PWM;
  }
 public JLabel get_brpwm_label(){
	 return  m_BR_PWM;
  }
 
 public JLabel get_pitch_label(){
	 return  m_PITCH;
  }
 
 public JLabel get_roll_label(){
	 return  m_ROLL;
  }
 
 public JLabel get_pitch_pid_sum_label(){
	 return  m_PITCH_PID_SUM;
  }
 
 public JLabel get_roll_pid_sum_label(){
	 return  m_ROLL_PID_SUM;
  }
 public JLabel get_xcoord_label(){
	 return  m_Xcoord;
  }
 
 
 public JButton get_offset_add(){
    return moffsetAdd;
 }
 
 public JButton get_offset_sub(){
	    return moffsetSub;
	 }
 
 public JTextField get_input_scale(){
	 
	 return mInputScale;
 }
 
public JTextField get_input_offset(){
	 
	 return mInputOffset;
 }
 
 public JLabel get_scale_label(){
	 return  mLInputscale;
  }
 
 public JLabel get_offset_label(){
	 return  mLInputOffset;
  }
 
 public  JCheckBox get_checkbox_FL(){
     return mchbox_FL;
 }
 
 public  JCheckBox get_checkbox_FR(){
     return mchbox_FR;
 }
 
 public  JCheckBox get_checkbox_BL(){
     return mchbox_BL;
 }
 
 public  JCheckBox get_checkbox_BR(){
     return mchbox_BR;
 }
 
 public  JCheckBox get_checkbox_GP(){
     return mchbox_G_PITCH;
 }
 
 public  JCheckBox get_checkbox_GR(){
     return mchbox_G_ROLL;
 }
 

 public static void main(String[] args) {
  //設定視窗的外觀
  JFrame.setDefaultLookAndFeelDecorated(true);


  JFrame frame = new JFrame("PID");
  
  PID  _pid = new PID();

  //在Swing的JFrame元件中，
  //要增加元件或是設定LayoutManager等動作，
  //要間接透過getContentPane()方法取得RootPane，
  //才能在上面進行動作。
 
  _pid.input_scale_init();

  frame.getContentPane().add(_pid.get_input_scale());
  frame.getContentPane().add(_pid.get_input_offset());
  frame.getContentPane().add(_pid.get_scale_label());
  frame.getContentPane().add(_pid.get_offset_label());
  frame.getContentPane().add(_pid.get_checkbox_FL());
  frame.getContentPane().add(_pid.get_checkbox_FR());
  frame.getContentPane().add(_pid.get_checkbox_BL());
  frame.getContentPane().add(_pid.get_checkbox_BR());
  frame.getContentPane().add(_pid.get_checkbox_GP());
  frame.getContentPane().add(_pid.get_checkbox_GR());
  
  frame.getContentPane().add(_pid.get_offset_add());
  frame.getContentPane().add(_pid.get_offset_sub());
  
  frame.getContentPane().add(_pid.get_flpwm_label());
  frame.getContentPane().add(_pid.get_frpwm_label());
  frame.getContentPane().add(_pid.get_blpwm_label());
  frame.getContentPane().add(_pid.get_brpwm_label());
  frame.getContentPane().add(_pid.get_pitch_label());
  frame.getContentPane().add(_pid.get_roll_label());
  frame.getContentPane().add(_pid.get_roll_pid_sum_label());
  frame.getContentPane().add(_pid.get_pitch_pid_sum_label());
  frame.getContentPane().add(_pid.get_xcoord_label());
  
  
  frame.getContentPane().add(_pid);
  frame.getContentPane().setBackground(Color.BLACK);

  //設定視窗顯示在螢幕在的位置
  frame.setLocation(0,0);

  //讓視窗右上角的X圖示被按下之後，視窗會關閉
  frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

  frame.pack();
  frame.setVisible(true);
 }
 
 @Override
 public void actionPerformed(ActionEvent ae) {
	
      
      input_scale = Integer.parseInt(mInputScale.getText());
      input_offset = Integer.parseInt(mInputOffset.getText());
      
      
      if(ae.getActionCommand()=="offset_add")
      {
    	  if(input_offset<(line_cnt-200))
    	  input_offset+=200;
    	  mInputOffset.setText(Integer.toString(input_offset));
    	  //System.out.println("offset_add");
      }
      else if(ae.getActionCommand()=="offset_sub"){
    	  if(input_offset>=200)
    	    input_offset-=200;
    	  mInputOffset.setText(Integer.toString(input_offset));
    	  //System.out.println("offset_add");
      }
      
      FL_EN = mchbox_FL.isSelected();
      FR_EN = mchbox_FR.isSelected();
      BL_EN = mchbox_BL.isSelected();
      BR_EN = mchbox_BR.isSelected();
      GP_EN = mchbox_G_PITCH.isSelected();
      GR_EN = mchbox_G_ROLL.isSelected();
      //System.out.println(FR_EN);
      
      
     
      
      validate();
      repaint();
 }
}

