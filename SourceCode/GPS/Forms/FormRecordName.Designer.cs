﻿
namespace AgOpenGPS.Forms
{
    partial class FormRecordName
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.panel1 = new System.Windows.Forms.Panel();
            this.lblFilename = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.tboxFieldName = new System.Windows.Forms.TextBox();
            this.panel2 = new System.Windows.Forms.Panel();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.checkBoxRecordAddTime = new System.Windows.Forms.CheckBox();
            this.buttonSave = new System.Windows.Forms.Button();
            this.checkBoxRecordAddDate = new System.Windows.Forms.CheckBox();
            this.buttonRecordCancel = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.cboxAddTime = new System.Windows.Forms.CheckBox();
            this.btnSave = new System.Windows.Forms.Button();
            this.cboxAddDate = new System.Windows.Forms.CheckBox();
            this.btnSerialCancel = new System.Windows.Forms.Button();
            this.panel1.SuspendLayout();
            this.panel2.SuspendLayout();
            this.SuspendLayout();
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.panel1.Controls.Add(this.lblFilename);
            this.panel1.Controls.Add(this.label1);
            this.panel1.Controls.Add(this.tboxFieldName);
            this.panel1.Controls.Add(this.panel2);
            this.panel1.Controls.Add(this.label3);
            this.panel1.Controls.Add(this.label2);
            this.panel1.Controls.Add(this.cboxAddTime);
            this.panel1.Controls.Add(this.btnSave);
            this.panel1.Controls.Add(this.cboxAddDate);
            this.panel1.Controls.Add(this.btnSerialCancel);
            this.panel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel1.Location = new System.Drawing.Point(2, 2);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(660, 256);
            this.panel1.TabIndex = 149;
            // 
            // lblFilename
            // 
            this.lblFilename.AutoSize = true;
            this.lblFilename.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.lblFilename.Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFilename.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.lblFilename.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.lblFilename.Location = new System.Drawing.Point(20, 97);
            this.lblFilename.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.lblFilename.Name = "lblFilename";
            this.lblFilename.Size = new System.Drawing.Size(83, 19);
            this.lblFilename.TabIndex = 153;
            this.lblFilename.Text = "Filename";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.label1.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.label1.Location = new System.Drawing.Point(20, 33);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(175, 23);
            this.label1.TabIndex = 152;
            this.label1.Text = "Enter Record Name";
            // 
            // tboxFieldName
            // 
            this.tboxFieldName.BackColor = System.Drawing.Color.AliceBlue;
            this.tboxFieldName.Font = new System.Drawing.Font("Tahoma", 18F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.tboxFieldName.Location = new System.Drawing.Point(13, 56);
            this.tboxFieldName.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.tboxFieldName.Name = "tboxFieldName";
            this.tboxFieldName.Size = new System.Drawing.Size(634, 36);
            this.tboxFieldName.TabIndex = 151;
            this.tboxFieldName.Click += new System.EventHandler(this.tboxFieldName_Click);
            this.tboxFieldName.TextChanged += new System.EventHandler(this.tboxFieldName_TextChanged);
            // 
            // panel2
            // 
            this.panel2.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.panel2.Controls.Add(this.label4);
            this.panel2.Controls.Add(this.label5);
            this.panel2.Controls.Add(this.checkBoxRecordAddTime);
            this.panel2.Controls.Add(this.buttonSave);
            this.panel2.Controls.Add(this.checkBoxRecordAddDate);
            this.panel2.Controls.Add(this.buttonRecordCancel);
            this.panel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel2.Location = new System.Drawing.Point(0, 0);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(660, 256);
            this.panel2.TabIndex = 154;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.label4.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.label4.Location = new System.Drawing.Point(175, 174);
            this.label4.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(24, 23);
            this.label4.TabIndex = 150;
            this.label4.Text = "+";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.label5.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.label5.Location = new System.Drawing.Point(18, 174);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(24, 23);
            this.label5.TabIndex = 149;
            this.label5.Text = "+";
            // 
            // checkBoxRecordAddTime
            // 
            this.checkBoxRecordAddTime.Appearance = System.Windows.Forms.Appearance.Button;
            this.checkBoxRecordAddTime.BackColor = System.Drawing.Color.Transparent;
            this.checkBoxRecordAddTime.FlatAppearance.CheckedBackColor = System.Drawing.Color.Teal;
            this.checkBoxRecordAddTime.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.checkBoxRecordAddTime.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.checkBoxRecordAddTime.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.checkBoxRecordAddTime.Image = global::AgOpenGPS.Properties.Resources.JobNameTime;
            this.checkBoxRecordAddTime.Location = new System.Drawing.Point(206, 150);
            this.checkBoxRecordAddTime.Name = "checkBoxRecordAddTime";
            this.checkBoxRecordAddTime.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.checkBoxRecordAddTime.Size = new System.Drawing.Size(86, 70);
            this.checkBoxRecordAddTime.TabIndex = 2;
            this.checkBoxRecordAddTime.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.checkBoxRecordAddTime.UseVisualStyleBackColor = false;
            // 
            // buttonSave
            // 
            this.buttonSave.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.buttonSave.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.buttonSave.FlatAppearance.BorderSize = 0;
            this.buttonSave.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.buttonSave.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.buttonSave.Image = global::AgOpenGPS.Properties.Resources.OK64;
            this.buttonSave.Location = new System.Drawing.Point(562, 150);
            this.buttonSave.Name = "buttonSave";
            this.buttonSave.Size = new System.Drawing.Size(83, 79);
            this.buttonSave.TabIndex = 3;
            this.buttonSave.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.buttonSave.UseVisualStyleBackColor = false;
            this.buttonSave.Click += new System.EventHandler(this.buttonSave_Click);
            // 
            // checkBoxRecordAddDate
            // 
            this.checkBoxRecordAddDate.Appearance = System.Windows.Forms.Appearance.Button;
            this.checkBoxRecordAddDate.BackColor = System.Drawing.Color.Transparent;
            this.checkBoxRecordAddDate.FlatAppearance.CheckedBackColor = System.Drawing.Color.Teal;
            this.checkBoxRecordAddDate.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.checkBoxRecordAddDate.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.checkBoxRecordAddDate.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.checkBoxRecordAddDate.Image = global::AgOpenGPS.Properties.Resources.JobNameCalendar;
            this.checkBoxRecordAddDate.Location = new System.Drawing.Point(49, 150);
            this.checkBoxRecordAddDate.Name = "checkBoxRecordAddDate";
            this.checkBoxRecordAddDate.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.checkBoxRecordAddDate.Size = new System.Drawing.Size(86, 70);
            this.checkBoxRecordAddDate.TabIndex = 1;
            this.checkBoxRecordAddDate.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.checkBoxRecordAddDate.UseVisualStyleBackColor = false;
            // 
            // buttonRecordCancel
            // 
            this.buttonRecordCancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.buttonRecordCancel.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.buttonRecordCancel.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.buttonRecordCancel.DialogResult = System.Windows.Forms.DialogResult.Cancel;
            this.buttonRecordCancel.FlatAppearance.BorderSize = 0;
            this.buttonRecordCancel.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.buttonRecordCancel.Font = new System.Drawing.Font("Tahoma", 12F);
            this.buttonRecordCancel.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.buttonRecordCancel.Image = global::AgOpenGPS.Properties.Resources.Cancel64;
            this.buttonRecordCancel.Location = new System.Drawing.Point(454, 149);
            this.buttonRecordCancel.Name = "buttonRecordCancel";
            this.buttonRecordCancel.Size = new System.Drawing.Size(77, 79);
            this.buttonRecordCancel.TabIndex = 4;
            this.buttonRecordCancel.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.buttonRecordCancel.UseVisualStyleBackColor = false;
            this.buttonRecordCancel.Click += new System.EventHandler(this.buttonRecordCancel_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.label3.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.label3.Location = new System.Drawing.Point(175, 174);
            this.label3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(24, 23);
            this.label3.TabIndex = 150;
            this.label3.Text = "+";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.label2.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.label2.Location = new System.Drawing.Point(18, 174);
            this.label2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(24, 23);
            this.label2.TabIndex = 149;
            this.label2.Text = "+";
            // 
            // cboxAddTime
            // 
            this.cboxAddTime.Appearance = System.Windows.Forms.Appearance.Button;
            this.cboxAddTime.BackColor = System.Drawing.Color.Transparent;
            this.cboxAddTime.FlatAppearance.CheckedBackColor = System.Drawing.Color.Teal;
            this.cboxAddTime.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.cboxAddTime.Font = new System.Drawing.Font("Tahoma", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxAddTime.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.cboxAddTime.Image = global::AgOpenGPS.Properties.Resources.JobNameTime;
            this.cboxAddTime.Location = new System.Drawing.Point(206, 150);
            this.cboxAddTime.Name = "cboxAddTime";
            this.cboxAddTime.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.cboxAddTime.Size = new System.Drawing.Size(86, 70);
            this.cboxAddTime.TabIndex = 2;
            this.cboxAddTime.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.cboxAddTime.UseVisualStyleBackColor = false;
            // 
            // btnSave
            // 
            this.btnSave.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnSave.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.btnSave.FlatAppearance.BorderSize = 0;
            this.btnSave.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSave.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.btnSave.Image = global::AgOpenGPS.Properties.Resources.OK64;
            this.btnSave.Location = new System.Drawing.Point(562, 150);
            this.btnSave.Name = "btnSave";
            this.btnSave.Size = new System.Drawing.Size(83, 79);
            this.btnSave.TabIndex = 3;
            this.btnSave.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnSave.UseVisualStyleBackColor = false;
            // 
            // cboxAddDate
            // 
            this.cboxAddDate.Appearance = System.Windows.Forms.Appearance.Button;
            this.cboxAddDate.BackColor = System.Drawing.Color.Transparent;
            this.cboxAddDate.FlatAppearance.CheckedBackColor = System.Drawing.Color.Teal;
            this.cboxAddDate.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.cboxAddDate.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cboxAddDate.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.cboxAddDate.Image = global::AgOpenGPS.Properties.Resources.JobNameCalendar;
            this.cboxAddDate.Location = new System.Drawing.Point(49, 150);
            this.cboxAddDate.Name = "cboxAddDate";
            this.cboxAddDate.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.cboxAddDate.Size = new System.Drawing.Size(86, 70);
            this.cboxAddDate.TabIndex = 1;
            this.cboxAddDate.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.cboxAddDate.UseVisualStyleBackColor = false;
            // 
            // btnSerialCancel
            // 
            this.btnSerialCancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnSerialCancel.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(50)))), ((int)(((byte)(50)))), ((int)(((byte)(70)))));
            this.btnSerialCancel.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btnSerialCancel.DialogResult = System.Windows.Forms.DialogResult.Cancel;
            this.btnSerialCancel.FlatAppearance.BorderSize = 0;
            this.btnSerialCancel.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSerialCancel.Font = new System.Drawing.Font("Tahoma", 12F);
            this.btnSerialCancel.ForeColor = System.Drawing.SystemColors.ButtonFace;
            this.btnSerialCancel.Image = global::AgOpenGPS.Properties.Resources.Cancel64;
            this.btnSerialCancel.Location = new System.Drawing.Point(454, 149);
            this.btnSerialCancel.Name = "btnSerialCancel";
            this.btnSerialCancel.Size = new System.Drawing.Size(77, 79);
            this.btnSerialCancel.TabIndex = 4;
            this.btnSerialCancel.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.btnSerialCancel.UseVisualStyleBackColor = false;
            // 
            // FormRecordName
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(96F, 96F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Dpi;
            this.BackColor = System.Drawing.Color.Cyan;
            this.ClientSize = new System.Drawing.Size(664, 260);
            this.ControlBox = false;
            this.Controls.Add(this.panel1);
            this.Font = new System.Drawing.Font("Tahoma", 14.25F);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
            this.Margin = new System.Windows.Forms.Padding(5);
            this.Name = "FormRecordName";
            this.Padding = new System.Windows.Forms.Padding(2);
            this.ShowInTaskbar = false;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterParent;
            this.Text = "Create New Record";
            this.Load += new System.EventHandler(this.FormRecordName_Load);
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.CheckBox cboxAddTime;
        private System.Windows.Forms.Button btnSave;
        private System.Windows.Forms.CheckBox cboxAddDate;
        private System.Windows.Forms.Button btnSerialCancel;
        private System.Windows.Forms.Label lblFilename;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox tboxFieldName;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.CheckBox checkBoxRecordAddTime;
        private System.Windows.Forms.Button buttonSave;
        private System.Windows.Forms.CheckBox checkBoxRecordAddDate;
        private System.Windows.Forms.Button buttonRecordCancel;
    }
}