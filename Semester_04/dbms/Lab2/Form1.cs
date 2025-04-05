using System;
using System.Collections.Generic;
using System.Data;
using System.Configuration;
using System.Data.SqlClient;
using System.Windows.Forms;
using System.Linq;

namespace DBMS
{
    public partial class Form1 : Form
    {
        private SqlConnection connection;
        private SqlDataAdapter parentAdapter, childAdapter;
        private DataSet parentDataSet = new DataSet(), childDataSet = new DataSet();
        private int selectedParentId = -1;

        private string parentTableName;
        private string parentIdColumn;
        private string childTableName;
        private string childIdColumn;
        private string foreignKeyColumn;
        private string[] childColumnNames;
        private string[] childColumnLabels;
        private string[] childColumnDataTypes;
        private string[] insertParameterNames;
        private List<TextBox> inputTextBoxes = new List<TextBox>();

        public Form1()
        {
            InitializeComponent();
            LoadConfigSettings();
        }

        private void LoadConfigSettings()
        {
            try
            {
                string connectionString = ConfigurationManager.ConnectionStrings["DefaultConnection"].ConnectionString;
                connection = new SqlConnection(connectionString);

                parentTableName = ConfigurationManager.AppSettings["ParentTableName"];
                parentIdColumn = ConfigurationManager.AppSettings["ParentIdColumn"];
                childTableName = ConfigurationManager.AppSettings["ChildTableName"];
                childIdColumn = ConfigurationManager.AppSettings["ChildIdColumn"];
                foreignKeyColumn = ConfigurationManager.AppSettings["ForeignKeyColumn"];

                childColumnNames = ConfigurationManager.AppSettings["ChildColumnNames"].Split(',');
                childColumnLabels = ConfigurationManager.AppSettings["ChildColumnLabels"].Split(',');
                childColumnDataTypes = ConfigurationManager.AppSettings["ChildColumnDataTypes"].Split(',');
                insertParameterNames = ConfigurationManager.AppSettings["InsertParameterNames"].Split(',');

                label5.Text = parentTableName;
                label6.Text = childTableName;

                ClearDynamicControls();

                GenerateDynamicInputFields();
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading configuration: " + ex.Message);
            }
        }

        private void ClearDynamicControls()
        {
            var controlsToRemove = Controls.OfType<Control>()
                .Where(c => c.Tag != null && c.Tag.ToString() == "DynamicControl")
                .ToList();

            foreach (var control in controlsToRemove)
            {
                Controls.Remove(control);
                control.Dispose();
            }

            inputTextBoxes.Clear();
        }

        private void GenerateDynamicInputFields()
        {
            int startX = 500;
            int startY = 440;
            int labelWidth = 80;
            int textboxWidth = 120;
            int rowHeight = 30;
            int maxControlsPerRow = 2;

            for (int i = 0; i < childColumnNames.Length; i++)
            {
                int row = i / maxControlsPerRow;
                int col = i % maxControlsPerRow;

                Label lbl = new Label();
                lbl.AutoSize = true;
                lbl.Location = new System.Drawing.Point(startX + col * (labelWidth + textboxWidth + 20), startY + row * rowHeight);
                lbl.Name = "dynamicLabel_" + i;
                lbl.Size = new System.Drawing.Size(labelWidth, 13);
                lbl.Text = childColumnLabels[i];
                lbl.Tag = "DynamicControl";
                Controls.Add(lbl);

                TextBox txt = new TextBox();
                txt.Location = new System.Drawing.Point(startX + labelWidth + 5 + col * (labelWidth + textboxWidth + 20), startY + row * rowHeight - 3);
                txt.Name = "dynamicTextBox_" + i;
                txt.Size = new System.Drawing.Size(textboxWidth, 20);
                txt.Tag = "DynamicControl";
                Controls.Add(txt);

                inputTextBoxes.Add(txt);
            }

            PerformLayout();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            LoadParentData();
        }

        private void LoadParentData()
        {
            try
            {
                parentAdapter = new SqlDataAdapter($"SELECT * FROM {parentTableName}", connection);
                parentDataSet.Clear();
                parentAdapter.Fill(parentDataSet);
                dataGridView1.DataSource = parentDataSet.Tables[0];
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error loading {parentTableName}: " + ex.Message);
            }
        }


        private void dataGridView1_CellClick(object sender, DataGridViewCellEventArgs e)
        {
            if (e.RowIndex >= 0)
            {
                selectedParentId = Convert.ToInt32(dataGridView1.Rows[e.RowIndex].Cells[parentIdColumn].Value);
                LoadChildData(selectedParentId);
            }
        }

        private void LoadChildData(int parentId)
        {
            try
            {
                childAdapter = new SqlDataAdapter($"SELECT * FROM {childTableName} WHERE {foreignKeyColumn} = @parentId", connection);
                childAdapter.SelectCommand.Parameters.AddWithValue("@parentId", parentId);

                childDataSet.Clear();
                childAdapter.Fill(childDataSet);
                dataGridView2.DataSource = childDataSet.Tables[0];
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error loading {childTableName}: " + ex.Message);
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
           
        }

        private void dataGridView1_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (selectedParentId == -1)
            {
                MessageBox.Show($"Please select a {parentTableName.TrimEnd('s')} first.");
                return;
            }

            try
            {
                string columnList = string.Join(",", childColumnNames);
                string paramList = string.Join(",", insertParameterNames);

                string insertQuery = string.Format(
                    ConfigurationManager.AppSettings["InsertQuery"],
                    childTableName,         
                    columnList,             
                    paramList,               
                    childIdColumn,           
                    foreignKeyColumn,       
                    foreignKeyColumn        
                );

                SqlCommand insertCmd = new SqlCommand(insertQuery, connection);

                for (int i = 0; i < childColumnNames.Length; i++)
                {
                    object paramValue;

                    switch (childColumnDataTypes[i].ToLower())
                    {
                        case "int":
                            paramValue = int.Parse(inputTextBoxes[i].Text);
                            break;
                        case "decimal":
                            paramValue = decimal.Parse(inputTextBoxes[i].Text);
                            break;
                        case "datetime":
                            paramValue = DateTime.Parse(inputTextBoxes[i].Text);
                            break;
                        default: // string and other types
                            paramValue = inputTextBoxes[i].Text;
                            break;
                    }

                    insertCmd.Parameters.AddWithValue(insertParameterNames[i], paramValue);
                }

                insertCmd.Parameters.AddWithValue("@foreignKey", selectedParentId);

                connection.Open();
                insertCmd.ExecuteNonQuery();
                connection.Close();

                MessageBox.Show($"{childTableName.TrimEnd('s')} added successfully!");
                LoadChildData(selectedParentId);
                ClearInputFields();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error inserting {childTableName.TrimEnd('s')}: " + ex.Message);
                if (connection.State == ConnectionState.Open)
                    connection.Close();
            }
        }

        private void ClearInputFields()
        {
            foreach (var textBox in inputTextBoxes)
            {
                textBox.Clear();
            }
        }



        private void button4_Click_1(object sender, EventArgs e)
        {
            try
            {
                SqlCommandBuilder builder = new SqlCommandBuilder(childAdapter);
                childAdapter.Update(childDataSet.Tables[0]);
                MessageBox.Show($"{childTableName} details updated successfully!");
                LoadChildData(selectedParentId);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error updating {childTableName}: " + ex.Message);
            }
        }

        private void button3_Click_1(object sender, EventArgs e)
        {
            if (dataGridView2.SelectedRows.Count > 0)
            {
                int childId = Convert.ToInt32(dataGridView2.SelectedRows[0].Cells[childIdColumn].Value);
                try
                {
                    string deleteQuery = string.Format(
                        ConfigurationManager.AppSettings["DeleteQuery"],
                        childTableName,   
                        childIdColumn    
                    );

                    SqlCommand deleteCmd = new SqlCommand(deleteQuery, connection);
                    deleteCmd.Parameters.AddWithValue("@id", childId);

                    connection.Open();
                    deleteCmd.ExecuteNonQuery();
                    connection.Close();

                    MessageBox.Show($"{childTableName.TrimEnd('s')} removed successfully!");
                    LoadChildData(selectedParentId);
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"Error removing {childTableName.TrimEnd('s')}: " + ex.Message);
                    if (connection.State == ConnectionState.Open)
                        connection.Close();
                }
            }
            else
            {
                MessageBox.Show($"Please select a {childTableName.TrimEnd('s')} to remove.");
            }
        }


        private void button3_Click(object sender, EventArgs e)
        {
           
        }

        private void button4_Click(object sender, EventArgs e) 
        {
           
        }
    }
}
