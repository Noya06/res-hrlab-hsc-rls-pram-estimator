clear; clc;

% =========================================================================
% 最適化の自動連続実行オーケストレータ
% =========================================================================

NUM_RUNS = 20; % 実行回数
OPTIMIZER = 'fmincon'; % 'bayesopt' または 'fmincon'
NUM_MULTISTART_POINTS = 10;

% 実行ごとに異なる乱数系列を使用する
rng('shuffle');

% --- ROS 2 ノードと通信の初期化 ---
try
    node = ros2node("/matlab_bayesopt_orchestrator");
catch
    clear node;
    node = ros2node("/matlab_bayesopt_orchestrator");
end

% パブリッシャの作成
pub_rho = ros2publisher(node, "/optimization/rho_cmd", "std_msgs/Float64MultiArray");
pub_reset = ros2publisher(node, "/optimization/reset_episode", "std_msgs/Bool");
pub_save = ros2publisher(node, "/optimization/save_csv_trigger", "std_msgs/Bool");

% サブスクライバの作成
sub_sim_done = ros2subscriber(node, "/optimization/simulation_done", "std_msgs/Bool");
sub_csv_saved = ros2subscriber(node, "/optimization/csv_saved", "std_msgs/Bool");

% --- 設定パスおよび比較用ファイルの選択 ---
data_root_dir = fullfile(getenv('HOME'), 'Programs/colcon_ws/data');
data_dir = fullfile(data_root_dir, 'datafiles');

human_filename = 'ref_60.0_10.0_0.5.csv'; 
human_csv_path = fullfile(data_root_dir, human_filename);

waypoints_filename = 'waypoints_.json';
waypoints_path = fullfile(data_root_dir, waypoints_filename);

if ~exist(human_csv_path, 'file')
    error('[ERROR] 参照ファイルが見つかりません: %s', human_csv_path);
end

% 1. 最適化対象変数（rho1, rho2）の定義（alpha / rho3 は 0.5 固定）
vars = [
    optimizableVariable('rho1', [0.1, 100.0], 'Type', 'real')
    optimizableVariable('rho2', [0.0, 50.0], 'Type', 'real')
    ];

% 初期評価点の設定
initialXTable = table(20.0, 5.0, 'VariableNames', {'rho1', 'rho2'});

% サマリーデータを保持するテーブルの準備
summary_table = table('Size', [NUM_RUNS, 6], ...
    'VariableTypes', {'double', 'double', 'double', 'double', 'double', 'double'}, ...
    'VariableNames', {'Run', 'rho1', 'rho2', 'rho3_fixed', 'Best_J', 'Elapsed_Sec'});

overall_start_time = datestr(now, 'yyyymmdd_HHMMSS');

fprintf('\n==================================================\n');
fprintf(' Starting Automatic Optimization Loop (%d Runs)\n', NUM_RUNS);
fprintf('==================================================\n');

% =========================================================================
% 最適化ループ
% =========================================================================
for run_idx = 1:NUM_RUNS
    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n');
    fprintf(' STARTING RUN %d / %d\n', run_idx, NUM_RUNS);
    fprintf('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n');
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    run_tag = sprintf('run_%02d_%s', run_idx, timestamp);
    
    % 目的関数の定義
    obj = @(x) evaluate_simulation(x, pub_rho, pub_reset, pub_save, sub_sim_done, sub_csv_saved, human_csv_path, data_dir);

    tic;
    switch lower(OPTIMIZER)
        case 'bayesopt'
            results = bayesopt(obj, vars, ...
                'NumSeedPoints', 5, ...
                'MaxObjectiveEvaluations', 60, ...
                'AcquisitionFunctionName', 'expected-improvement-plus', ...
                'IsObjectiveDeterministic', true, ...
                'InitialX', initialXTable, ...
                'UseParallel', false, ...
                'PlotFcn', @plotCurrentAndMinJ);

            bestRho_opt = table2array(results.XAtMinObjective);
            bestJ = results.MinObjective;

        case 'fmincon'
            % --- MultiStart による広域探索の設定 ---
            lower_bounds = [vars(1).Range(1), vars(2).Range(1)];
            upper_bounds = [vars(1).Range(2), vars(2).Range(2)];
            start_points = generateRandomStartPoints( ...
                NUM_MULTISTART_POINTS, lower_bounds, upper_bounds, [5.0, 0.5]);
            x0 = start_points(1, :);

            fmincon_options = optimoptions('fmincon', ...
                'Algorithm', 'sqp', ...
                'DiffMinChange', 2.0, ...
                'DiffMaxChange', 20.0, ...
                'Display', 'off');

            obj_fmincon = @(x) evaluate_simulation( ...
                table(x(1), x(2), 'VariableNames', {'rho1', 'rho2'}), ...
                pub_rho, pub_reset, pub_save, sub_sim_done, sub_csv_saved, ...
                human_csv_path, data_dir);

            problem = createOptimProblem('fmincon', ...
                'objective', obj_fmincon, ...
                'x0', x0, ...
                'lb', lower_bounds, ...
                'ub', upper_bounds, ...
                'options', fmincon_options);

            ms = MultiStart('Display', 'iter');
            start_point_set = CustomStartPointSet(start_points);
            
            [bestRho_opt, bestJ] = run(ms, problem, start_point_set);

        otherwise
            error('Unsupported optimizer "%s". Use "bayesopt" or "fmincon".', OPTIMIZER);
    end
    elapsed_time = toc;

    % 最適化経過グラフの自動保存（bayesopt の場合のみ）
    if strcmpi(OPTIMIZER, 'bayesopt')
        try
            fig_bayes = findobj('Type', 'figure', 'Number', 100);
            if ~isempty(fig_bayes)
                progress_path = fullfile(data_dir, ...
                    sprintf('%s_progress_%s.png', OPTIMIZER, run_tag));
                saveas(fig_bayes, progress_path);
                fprintf('[INFO] 最適化経過グラフを保存しました: %s\n', progress_path);
            end
        catch ME
            warning('[WARN] 経過グラフの保存に失敗しました: %s', ME.message);
        end
    end

    % 結果の取得とサマリー配列への記録
    if isfinite(bestJ) && bestJ < 1e5 && numel(bestRho_opt) == 2 && all(isfinite(bestRho_opt))
        bestRho = [bestRho_opt(1), bestRho_opt(2), 0.5]; % alpha = 0.5 固定
        
        fprintf('\n-----------------------------------------\n');
        fprintf('Run %d Completed!\n', run_idx);
        fprintf('Best rho = [%.4f, %.4f, %.4f]\n', bestRho(1), bestRho(2), bestRho(3));
        fprintf('Best J   = %.6f\n', bestJ);
        fprintf('-----------------------------------------\n');

        % 各回の結果ファイルを保存
        save_best_parameters(bestRho, bestJ, human_filename, elapsed_time, run_tag, data_dir);
        
        % plot_optimization_result が同フォルダ・パス上に存在する場合のみ実行
        if exist('plot_optimization_result', 'file') == 2
            plot_optimization_result(human_csv_path, waypoints_path, run_tag, data_dir);
        end
        
        % サマリーテーブルに登録
        summary_table(run_idx, :) = {run_idx, bestRho(1), bestRho(2), bestRho(3), bestJ, elapsed_time};
    else
        fprintf('\n[WARN] Run %d: No feasible points found.\n', run_idx);
        summary_table(run_idx, :) = {run_idx, NaN, NaN, 0.0, Inf, elapsed_time};
    end
    
    close all; % メモリ開放および描画ウィンドウのクリーンアップ
end

% =========================================================================
% 全試行分のまとめ (Summary) ファイルの出力
% =========================================================================
summary_csv_path = fullfile(data_dir, sprintf('%s_summary_%druns_%s.csv', ...
    OPTIMIZER, NUM_RUNS, overall_start_time));
writetable(summary_table, summary_csv_path);

summary_txt_path = fullfile(data_dir, sprintf('%s_summary_%druns_%s.txt', ...
    OPTIMIZER, NUM_RUNS, overall_start_time));
fid = fopen(summary_txt_path, 'w');
if fid ~= -1
    fprintf(fid, "==================================================\n");
    fprintf(fid, " Optimization Summary (%d Runs Report)\n", NUM_RUNS);
    fprintf(fid, " Generated: %s\n", datestr(now));
    fprintf(fid, "==================================================\n\n");
    fprintf(fid, "Run |   rho1 (k_2)   |   rho2 (k_3)   | rho3 (alpha) |    Best J    | Time (s)\n");
    fprintf(fid, "----+----------------+----------------+--------------+--------------+----------\n");
    for i = 1:NUM_RUNS
        fprintf(fid, "%2d  | %14.6f | %14.6f | %12.4f | %12.8f | %8.2f\n", ...
            summary_table.Run(i), summary_table.rho1(i), summary_table.rho2(i), ...
            summary_table.rho3_fixed(i), summary_table.Best_J(i), summary_table.Elapsed_Sec(i));
    end
    fclose(fid);
end

fprintf('\n==================================================\n');
fprintf(' All %d runs completed successfully!\n', NUM_RUNS);
fprintf(' Summary CSV saved: %s\n', summary_csv_path);
fprintf(' Summary TXT saved: %s\n', summary_txt_path);
fprintf('==================================================\n');

clear node; % 終了時にノードを解放

% =========================================================================
% 評価関数（異常時・タイムアウト時は大きなペナルティ値を返して安全に継続）
% =========================================================================
function J = evaluate_simulation(x, pub_rho, pub_reset, pub_save, sub_sim_done, sub_csv_saved, human_csv_path, data_dir)
    alpha_fixed = 0.0;
    rho = [x.rho1, x.rho2, alpha_fixed];
    fprintf('\n--------------------------------------------------\n');
    fprintf('Evaluating rho = [%.4f, %.4f, %.4f]\n', rho(1), rho(2), rho(3));
    
    % fmincon がクラッシュするのを防ぐため、Inf/NaN の代わりに巨大なペナルティ値を定義
    PENALTY_VALUE = 1e6;
    J = PENALTY_VALUE;
    
    try
        msg_rho = ros2message(pub_rho);
        msg_rho.data = rho;
        send(pub_rho, msg_rho);
        fprintf('[PUB] Sent /optimization/rho_cmd\n');
        
        fprintf('[WAIT] Waiting for C++ nodes to complete simulation...\n');
        [~, status] = receive(sub_sim_done, 180);
        if ~status
            warning('[WARN] Timeout waiting for simulation_done.');
            J = PENALTY_VALUE;
            return;
        end
        fprintf('[STATE] Simulation done flag received.\n');
        
        msg_reset = ros2message(pub_reset);
        msg_reset.data = true;
        send(pub_reset, msg_reset);
        
        msg_save = ros2message(pub_save);
        msg_save.data = true;
        send(pub_save, msg_save);
        fprintf('[PUB] Sent /optimization/save_csv_trigger\n');
        
        % タイムアウト待ち時間を 15 秒から 30 秒に緩和
        [~, status] = receive(sub_csv_saved, 15);
        if ~status
            warning('[WARN] Timeout waiting for csv_saved.');
            J = PENALTY_VALUE;
            return;
        end
        fprintf('[STATE] CSV saved flag received.\n');
        
        J_calc = calculate_mse(human_csv_path, data_dir);
        
        if isinf(J_calc) || isnan(J_calc)
            fprintf('[WARN] 評価値 J が Inf/NaN のためペナルティ適用。\n');
            J = PENALTY_VALUE;
        else
            J = J_calc;
            fprintf('[SUCCESS] Evaluated J = %.6f\n', J);
        end
        
    catch ME
        warning('[ERROR] Error during evaluation: %s', ME.message);
        J = PENALTY_VALUE;
    end
    drawnow;
end

% =========================================================================
% MSE計算関数 (phi_hum_y と phi_com_y_clamped の平均二乗誤差)
% =========================================================================
function J = calculate_mse(human_csv_path, data_dir)
    J = Inf;
    files = dir(fullfile(data_dir, 'result_*.csv'));
    if isempty(files)
        warning('[ERROR] No simulation CSV file found in %s', data_dir);
        return;
    end
    
    [~, idx] = max([files.datenum]);
    latest_csv = fullfile(data_dir, files(idx).name);
    
    opts_hum = detectImportOptions(human_csv_path, 'CommentStyle', '#');
    opts_com = detectImportOptions(latest_csv, 'CommentStyle', '#');
    
    df_hum = readtable(human_csv_path, opts_hum);
    df_com = readtable(latest_csv, opts_com);
    
    n_samples = min(height(df_hum), height(df_com));
    if n_samples == 0
        warning('[ERROR] CSV file is empty.');
        return;
    end
    
    phi_hum_y = df_hum.phi_hum_y(1:n_samples);
    phi_com_y_raw = df_com.phi_com_y(1:n_samples);
    
    % 生のモデル出力 phi_com_y を 30 で割り、[-0.5, 0.5] にクランプして phi_hum_y とスケールを統一
    phi_com_y_scaled = phi_com_y_raw / 30.0;
    phi_com_y_clamped = min(max(phi_com_y_scaled, -0.5), 0.5);
    
    diff_phi = phi_hum_y - phi_com_y_clamped;
    J = mean(diff_phi.^2);
end

% =========================================================================
% 各回の履歴保存関数
% =========================================================================
function save_bayesopt_history(results, run_tag, data_dir)
    try
        history_table = results.XTrace;
        history_table.Objective = results.ObjectiveTrace;
        
        history_csv_path = fullfile(data_dir, sprintf('bayesopt_history_%s.csv', run_tag));
        writetable(history_table, history_csv_path);
        fprintf('[INFO] 探索履歴を保存しました: %s\n', history_csv_path);
    catch ME
        warning('[WARN] 探索履歴の保存に失敗しました: %s', ME.message);
    end
end

% =========================================================================
% 各回の最終パラメータ保存関数
% =========================================================================
function save_best_parameters(bestRho, bestJ, human_filename, elapsed_time, run_tag, data_dir)
    % 保存対象となる構造体を定義
    best_params_struct.bestRho = bestRho;
    best_params_struct.bestJ = bestJ;
    best_params_struct.human_filename = human_filename;
    best_params_struct.elapsed_time = elapsed_time;
    best_params_struct.run_tag = run_tag;

    % .mat ファイルへの保存
    mat_path = fullfile(data_dir, sprintf('best_params_%s.mat', run_tag));
    save(mat_path, 'best_params_struct');
    fprintf('[INFO] パラメータMATファイルを保存しました: %s\n', mat_path);

    % .txt ファイルへの保存
    txt_path = fullfile(data_dir, sprintf('best_params_%s.txt', run_tag));
    fid = fopen(txt_path, 'w');
    if fid ~= -1
        fprintf(fid, "=========================================\n");
        fprintf(fid, " Optimization Result Summary (%s)\n", run_tag);
        fprintf(fid, " Date/Time: %s\n", datestr(now));
        fprintf(fid, "=========================================\n");
        fprintf(fid, "Reference CSV : %s\n", human_filename);
        fprintf(fid, "Best J (MSE)  : %.8f\n", bestJ);
        fprintf(fid, "Elapsed Time  : %.2f sec\n\n", elapsed_time);
        fprintf(fid, "Optimal Parameters:\n");
        fprintf(fid, "  rho1 (k_2)   : %.6f\n", bestRho(1));
        fprintf(fid, "  rho2 (k_3)   : %.6f\n", bestRho(2));
        fprintf(fid, "  rho3 (alpha) : %.6f (fixed)\n", bestRho(3));
        fclose(fid);
        fprintf('[INFO] パラメータテキストを保存しました: %s\n', txt_path);
    end
end

% =========================================================================
% プロット関数
% =========================================================================
function stop = plotCurrentAndMinJ(results, state)
    stop = false;

    if isempty(results.ObjectiveTrace)
        return;
    end

    objectiveValues = results.ObjectiveTrace(:);
    nEvaluations = numel(objectiveValues);
    evaluationNumbers = (1:nEvaluations)';

    minValues = nan(size(objectiveValues));
    bestSoFar = Inf;
    for i = 1:nEvaluations
        value = objectiveValues(i);
        if ~isnan(value) && value < bestSoFar
            bestSoFar = value;
        end
        if isfinite(bestSoFar)
            minValues(i) = bestSoFar;
        end
    end

    fig = figure(100);
    clf(fig);
    set(fig, 'Name', 'Current J and Minimum J', 'NumberTitle', 'off', 'Color', 'w');
    ax = axes('Parent', fig);

    plot(ax, evaluationNumbers, objectiveValues, 'b-o', 'LineWidth', 1.2, 'MarkerSize', 4, 'DisplayName', 'Current J');
    hold(ax, 'on');
    plot(ax, evaluationNumbers, minValues, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Minimum J');

    grid(ax, 'on');
    xlabel(ax, 'Function evaluations');
    ylabel(ax, 'Objective J');
    title(ax, sprintf('Current J and Minimum J (%s)', state));
    legend(ax, 'Location', 'best');
    xlim(ax, [1, max(2, nEvaluations)]);
    hold(ax, 'off');
    drawnow;
end

function start_points = generateRandomStartPoints(count, lower_bounds, upper_bounds, minimum_separation)
    start_points = zeros(count, numel(lower_bounds));
    max_attempts = 100000;

    for point_idx = 1:count
        accepted = false;

        for attempt = 1:max_attempts
            candidate = lower_bounds + ...
                (upper_bounds - lower_bounds) .* rand(1, numel(lower_bounds));

            if point_idx == 1
                accepted = true;
            else
                rho1_separated = abs(start_points(1:point_idx - 1, 1) - candidate(1)) ...
                    >= minimum_separation(1);
                rho2_separated = abs(start_points(1:point_idx - 1, 2) - candidate(2)) ...
                    >= minimum_separation(2);
                accepted = all(rho1_separated & rho2_separated);
            end

            if accepted
                start_points(point_idx, :) = candidate;
                break;
            end
        end

        if ~accepted
            error('ランダムなMultiStart開始点を生成できませんでした。範囲または分離条件を確認してください。');
        end
    end
end