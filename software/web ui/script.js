const API_URL = "http://127.0.0.1:5000/data";


let rawData = [];
let refreshTimerId = null;
let chart = null;

const tableBody = document.getElementById("data-table-body");
const refreshSelect = document.getElementById("refresh-interval-select");
const chartCanvas = document.getElementById('water-chart');

async function getData() {
    const response = await fetch(API_URL);
    const json = await response.json();
    rawData = json
      .map(r => ({
        timestamp: Number(r.timestamp),
        value: Number(r.value)
      }));
      
    renderTable();
    renderChart();
}

function renderTable() {
  tableBody.innerHTML = "";
  for (const item of rawData) {
    const tr = document.createElement("tr");

    const dateCell = document.createElement("td");
    const date = new Date(item.timestamp * 1000);
    dateCell.textContent = date.toLocaleString();

    const valueCell = document.createElement("td");
    valueCell.textContent = item.value;

    tr.appendChild(dateCell);
    tr.appendChild(valueCell);
    tableBody.appendChild(tr);
  }
}

function setAutoRefresh() {
  const interval = parseInt(refreshSelect.value, 10);
  if (refreshTimerId) {
    clearInterval(refreshTimerId);
    refreshTimerId = null;
  }
  if (interval > 0) {
    refreshTimerId = setInterval(getData, interval);
  }
}

function renderChart() {

  const labels = rawData.map(item =>
    new Date(item.timestamp * 1000).toLocaleTimeString()
  );
  const values = rawData.map(item => item.value);

  if (!chart) {
    chart = new Chart(chartCanvas, {
      type: 'line',
      data: {
        labels: labels,
        datasets: [{
          label: 'Wasserstand in Liter',
          data: values,
          borderWidth: 1,
          responsive: true,
          maintainAspectRatio: false,
        }]
      },
      options: {
        scales: {
          x: {
            ticks: {
              color: "white",
              maxRotation: 0,
              autoSkip: true,
              maxTicksLimit: 6,
              font: {
                size: 11
              }
            }
          },
          y: {
            beginAtZero: true,
            grid: { color: "#1f2937" },
            ticks: {
              color: "white",
              font: {
                size: 11
              }
            }
          }
        },
        plugins: {
            tooltip: {
              mode: "index",
              intersect: false
            },
            legend: {
              labels: {
                color: "white",
                font: { size: 11 }
              }
            }
          }
      }
    });
  } else {
    chart.data.labels = labels;
    chart.data.datasets[0].data = values;
    chart.update();
  }
}


refreshSelect.addEventListener("change", setAutoRefresh);

getData();